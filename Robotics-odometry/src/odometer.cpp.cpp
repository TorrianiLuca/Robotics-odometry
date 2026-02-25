#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <cmath>

// global parameters
double  rear_wheels_baseline = 1.30; //[m]
double d = 1.765; //[m]
double steering_factor = 32.0;
double x = 0.0, y = 0.0, theta_t1 = 0.0 , theta_t0 = 0.0;
double steer_deg, speed_kmh;
bool init = true;
static ros::Time previous_time, current_time;

void callback(const geometry_msgs::PointStamped::ConstPtr& msg){

    if(init){
        previous_time = ros::Time::now();
        init = false;
    }

    steer_deg = msg->point.x;
    speed_kmh = msg->point.y;

    current_time = msg->header.stamp;
    double dt = (current_time - previous_time).toSec(); 
    if (dt <= 0.0) {
        return;
    }
         
    previous_time = current_time;

    double speed_ms = speed_kmh / 3.6; //[m/s]
    double steer_rad = (steer_deg / steering_factor) * M_PI /180.0; // rad

    double linear_velocity = speed_ms;
    //double dtheta = theta_t1 - theta_t0;
    double angular_velocity = (linear_velocity / d ) * tan(steer_rad) ; // bicycle model  
    
    theta_t0 = theta_t1;

    //odometry: orientation and position
    theta_t1 += angular_velocity * dt;
    //theta_t1 = std::fmod(theta_t1 + M_PI, 2.0 * M_PI);

    if(std::fabs(angular_velocity) > 0.001){
        x += linear_velocity / angular_velocity * (sin(theta_t1) - sin(theta_t0)) / 20;
        y += linear_velocity / angular_velocity * (-cos(theta_t1) + cos(theta_t0)) / 20;
    }else{
        x += linear_velocity * cos(theta_t0) * dt / 20;
        y += linear_velocity * sin(theta_t0) * dt / 20;
    }

    static ros::NodeHandle n;
    static ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);

    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "vehicle";

    // update position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    //update orientation, the quaternion based on the new theta value
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_t1);
    odom.pose.pose.orientation = odom_quat;

    //velocity
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = angular_velocity;

    odom_pub.publish(odom);

    static ros::Time last_tf_time;

    // tf odom-vehicle
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, theta_t1);
    transform.setRotation(q);
    if(current_time != last_tf_time){
        br.sendTransform(tf::StampedTransform(transform, current_time, "odom", "vehicle"));   
        last_tf_time = current_time;     
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/speedsteer", 10, callback);
    ros::spin();

    return 0;
}
