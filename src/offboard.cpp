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
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#include <drone_img_proc/LandTarget.h>


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
std::vector<drone_img_proc::LandTarget> feedbacks;

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

void feedback_cb(const drone_img_proc::LandTarget::ConstPtr& msg){
    feedbacks.push_back(*msg);
}

int main(int argc, char **argv)
{
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
    ros::Subscriber home_pos_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("/mavros/home_position/home",1,home_cb);
    ros::Subscriber image_feedback_sub = nh.subscribe<drone_img_proc::LandTarget>
            ("/image_converter/image_feedback",10, feedback_cb);

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

    drone_img_proc::LandTarget nav_target;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = flight_altitude;

    //send a few setpoints before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    int i = 0;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    ros::Time last_request = ros::Time::now();
    bool within_area = true;
    bool will_land = false;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = flight_altitude;
    
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !will_land){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)) && !will_land){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        mavros_msgs::CommandTOL land_comm;
        geometry_msgs::PoseStamped loc_pos;
        //ROS_INFO("CURRENT POSITION: %f %f %f"
        //,current_position.pose.position.x
        //,current_position.pose.position.y
        //,current_position.pose.position.z);

        if(current_state.armed){
            for(; ros::ok() && i < 200; ++i){
                pose.pose.position.x = (RADIUS / 200) * ((double)i);
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
        }


        /*geometry_msgs::PoseStamped mark_pose;
        mark_pose.pose.position.x = 9.0;
        mark_pose.pose.position.y = 12.0;
        mark_pose.pose.position.z = flight_altitude;
        local_pos_pub.publish(mark_pose);*/
        /*ROS_INFO("CURRENT POSITION:%f %f %f",
        current_position.pose.position.x,
        current_position.pose.position.y,
        current_position.pose.position.z);*/

        if(feedbacks.size() > 0 && ros::Time::now() - feedbacks.back().stamp < ros::Duration(1.0)){
            drone_img_proc::LandTarget current_feedback = feedbacks.back();
            feedbacks.pop_back();
            ROS_INFO("RECEIVED FEEDBACK %d",current_feedback.centerFound);
        }
        
        
        if(abs(current_position.pose.position.z - flight_altitude) < 0.3){
            loc_pos.pose.position.z = flight_altitude;
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
            local_pos_pub.publish(pose);
        }

        if(circle_count == LAPS_BEFORE_IMG && !will_land){
                if(land_client.call(land_comm) && land_comm.response.success){
                    ROS_INFO("LAND COMMAND ACCEPTED");
                    will_land = true;
                }
        }
        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}