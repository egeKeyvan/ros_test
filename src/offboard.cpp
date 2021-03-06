/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandTOLRequest.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>


#define PI 3.14159
#define STEP_COUNT 720
#define e_neighbour(a,b) (abs(a) < b)

double RADIUS = 1.5;
double flight_altitude = 0;
int LAPS_BEFORE_IMG = 2;

int step_number;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_position;
mavros_msgs::HomePosition global_home;
mavros_msgs::PositionTarget tar;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double abs(double x){
    if(x < 0.0){
        return -x;
    }

    return x;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

void home_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    global_home = *msg;
}

int main(int argc, char **argv)
{
    uint32_t seq_count = 1;
    step_number = 0;
    int circle_count = 0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose",10,pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/land");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/takeoff");
    ros::Subscriber home_pos_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("/mavros/home_position/home",1,home_cb);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ROS_INFO("RECEIVING PARAMETERS");

    if(nh.getParam("lap_count_before_img",LAPS_BEFORE_IMG)){
        ROS_INFO("GOT PARAMETER LAPS_BEFORE_IMG:%d",LAPS_BEFORE_IMG);
    }else{
        ROS_ERROR("CANT RECEIVE LAP COUNT PARAMETER. ABORTING !!!");
        return 0;
    }

    if(nh.getParam("radius",RADIUS)){
        ROS_INFO("GOT PARAMETER RADIUS:%f",RADIUS);
    }else{
        ROS_ERROR("CANT RECEIVE RADIUS PARAMETER. ABORTING !!!");
        return 0;
    }

    if(nh.getParam("flight_altitude",flight_altitude)){
        ROS_INFO("GOT PARAMETER FLIGHT ALTITUDE:%f",flight_altitude);
    }else{
        ROS_ERROR("CANT RECEIVE FLIGHT ALTITUDE PARAMETER. ABORTING !!!");
        return 0;
    }

    ROS_INFO("ALL PARAMETERS RECEIVED");
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = flight_altitude;
    

    //send a few setpoints before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        pose.header.seq = seq_count++;
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    int i = 0;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandTOL takeoff_msg;
    arm_cmd.request.value = true;
    ros::Time takeoff_timer = ros::Time::now();
    ros::Time last_request = ros::Time::now();
    bool takeoff = false;
    while(ros::ok() && !takeoff){
        if(ros::Time::now() - last_request > ros::Duration(1.0)){
            takeoff_msg.request.altitude = global_home.geo.altitude + flight_altitude;
            takeoff_msg.request.latitude = global_home.geo.latitude;
            takeoff_msg.request.longitude = global_home.geo.longitude;
            takeoff_msg.request.min_pitch = 0.0;
            takeoff_msg.request.yaw = 0.0;
            if(takeoff_client.call(takeoff_msg) && takeoff_msg.response.success){
                ROS_INFO("SUCCESSFULLY ENGAGED TAKEOFF");
            }

            last_request = ros::Time::now();
        }

        if(current_state.armed){
            ROS_INFO("ARMED. TAKEOFF WILL FOLLOW");
            takeoff = true;
            takeoff_timer = ros::Time::now();
        }

        pose.header.seq = seq_count++;
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


    last_request = ros::Time::now();
    bool within_area = true;
    bool will_land = false;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = flight_altitude;
    bool offboard = false;
    

    while(ros::ok() && !offboard 
    && ros::Time::now() - takeoff_timer > ros::Duration(9.0) 
    && e_neighbour(current_position.pose.position.z - flight_altitude, 0.40)){
        if(current_state.mode == "OFFBOARD"){
            ROS_INFO("ALREADY ON OFFBOARD MODE");
            offboard = true;
            break;
        }
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(0.5))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                offboard = true;
            }
            last_request = ros::Time::now();
        }

        pose.header.seq = seq_count++;
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep(); 
    }


    bool first_time = true;
    ros::Time flight_timer = ros::Time::now();
    last_request = ros::Time::now();
    while(ros::ok()){
        mavros_msgs::CommandTOL land_msg;
        geometry_msgs::PoseStamped loc_pos;

        if(current_state.armed){
            for(; ros::ok() && i < 200; ++i){
                pose.pose.position.x = (RADIUS / 200) * ((double)i);
                pose.pose.position.y = 0.0;
                pose.pose.position.z = flight_altitude;
                pose.header.seq = seq_count++;
                pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(pose);
            }

            if(first_time){
                first_time = false;
                flight_timer = ros::Time::now();
            }
        }


        if(!first_time && !will_land && ros::Time::now() - flight_timer > ros::Duration(20.0) && ros::Time::now() - last_request > ros::Duration(1.0)){
            land_msg.request.altitude = global_home.geo.altitude;
            land_msg.request.latitude = global_home.geo.latitude;
            land_msg.request.longitude = global_home.geo.longitude;
            land_msg.request.min_pitch = 0.0;
            land_msg.request.yaw = 0.0;
            if(land_client.call(land_msg) && land_msg.response.success){
                ROS_INFO("SUCCESSFULLY ENGAGED LANDING");
                will_land = true;
            }

            last_request = ros::Time::now();
        }else{
            pose.pose.position.x = RADIUS;
            pose.pose.position.y = 0;
            pose.pose.position.z = flight_altitude;
            pose.header.seq = seq_count++;
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);
        }

        /*if(current_state.armed){
            if(first_time){
                first_time = false;
            }
            
            pose.header.seq = seq_count++;
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep(); 
        }

        if(!first_time && !will_land && ros::Time::now() - flight_timer > ros::Duration(10.0) && ros::Time::now() - last_request > ros::Duration(1.0)){
            land_msg.request.altitude = global_home.geo.altitude;
            land_msg.request.latitude = global_home.geo.latitude;
            land_msg.request.longitude = global_home.geo.longitude;
            land_msg.request.min_pitch = 0.0;
            land_msg.request.yaw = 0.0;
            if(land_client.call(land_msg) && land_msg.response.success){
                ROS_INFO("SUCCESSFULLY ENGAGED LANDING");
                will_land = true;
            }

            last_request = ros::Time::now();
        }*/

        //ROS_INFO("CURRENT POSITION: %f %f %f"
        //,current_position.pose.position.x
        //,current_position.pose.position.y
        //,current_position.pose.position.z);

        /*if(current_state.armed){
            for(; ros::ok() && i < 200; ++i){
                pose.pose.position.x = (RADIUS / 200) * ((double)i);
                local_pos_pub.publish(pose);
                if(!first_time){
                    first_time = true;
                    flight_time = ros::Time::now();
                }
        }else{
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0;
            pose.header.stamp = ros::Time::now();
            pose.header.seq = seq_count++,
            local_pos_pub.publish(pose);
        }

        

            req_timer = ros::Time::now();
        }
        
        ros::spinOnce();
        rate.sleep();

        /*geometry_msgs::PoseStamped mark_pose;
        mark_pose.pose.position.x = 9.0;
        mark_pose.pose.position.y = 12.0;
        mark_pose.pose.position.z = flight_altitude;
        local_pos_pub.publish(mark_pose);*/
        /*ROS_INFO("CURRENT POSITION:%f %f %f",
        current_position.pose.position.x,
        current_position.pose.position.y,
        current_position.pose.position.z);
        
        
        /*if(abs(current_position.pose.position.z - flight_altitude) < 0.3){
            loc_pos.pose.position.z = flight_altitude;
            loc_pos.header.stamp = ros::Time::now();
            loc_pos.header.seq = seq_count++;
            step_number = step_number % STEP_COUNT;
            step_number++;
            double current_angle = (PI / STEP_COUNT) * 2 * step_number;
            loc_pos.pose.position.x = RADIUS * cos(current_angle);
            loc_pos.pose.position.y = RADIUS * sin(current_angle);
            local_pos_pub.publish(loc_pos);

            if(e_neighbour(current_position.pose.position.x - RADIUS,0.3) && e_neighbour(current_position.pose.position.y,0.3)){
                if(!within_area){
                    ROS_INFO("LAP COUNT INCREASED");
                    circle_count++;
                }
                within_area = true;
            }else{
                within_area = false;
            }

        }else{
            //ROS_WARN("GAINING ALTITUDE");
            pose.header.seq = seq_count++;
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);
        }

        if(circle_count == LAPS_BEFORE_IMG && !will_land){
                if(land_client.call(land_comm) && land_comm.response.success){
                    ROS_INFO("LAND COMMAND ACCEPTED");
                    will_land = true;
                }
        }
        ros::spinOnce();
        rate.sleep();*/
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}