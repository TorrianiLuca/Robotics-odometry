#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

//global parameters
double a = 6378137.0;
double b = 6356752.0;
double e_squared = 1 - (( b*b ) /  ( a*a )); 
bool init = true;
double lat_r, lon_r, alt_r;
double x_r, y_r, z_r;
double phi_r, lambda_r;
double x_previous = 0.0, y_previous = 0.0, heading = 0.0;

void callback(const sensor_msgs::NavSatFix::ConstPtr& msg){

    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    //convert lla to cartedian ECEF
    double phi =  lat * M_PI / 180.0; //lat_rad
    double lambda = lon * M_PI / 180.0; //lon_rad
    double h = alt;

    double N = a / (sqrt(1 - e_squared * sin(phi)*sin(phi)));
    
    double x_ecef = (N + h) * cos(phi) * cos(lambda) /20;
    double y_ecef = (N + h) * cos(phi) * sin(lambda) /20;
    double z_ecef = (N * (1 - e_squared) + h) * sin(phi) /20;

    //convert cartesian ECEF to ENU

    if(init){
        //set reference point to the first value from gps
        lat_r = lat;
        lon_r = lon;
        alt_r = alt;

        x_r = x_ecef;
        y_r = y_ecef;
        z_r = z_ecef;

        phi_r = phi;
        lambda_r = lambda;

        init = false;
    }

    double dx = x_ecef - x_r;
    double dy = y_ecef - y_r;
    double dz = z_ecef - z_r;

    double x_enu = -sin(lambda_r) * dx + cos(lambda_r) * dy /20; 
    double y_enu = -sin(phi_r) * cos(lambda_r) * dx - sin(phi_r) * sin(lambda_r) * dy + cos(phi_r) * dz /20;
    double z_enu = cos(phi_r) * cos(lambda_r) * dx + cos(phi_r) * sin(lambda_r) * dy + sin(phi_r) * dz;

    //heading
    if(x_previous != x_enu && y_previous != y_enu){
        heading = atan2(y_enu - y_previous, x_enu - x_previous); // computing heading from last and actual position from gps
    }

    y_previous = y_enu;
    x_previous = x_enu;

    //publishing
    static ros::NodeHandle n;
    static ros::Publisher gps_odom_pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 10);

    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "gps";

    odom.pose.pose.position.x = x_enu;
    odom.pose.pose.position.y = y_enu;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);

    gps_odom_pub.publish(odom);

     // tf odom-gps
     static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin(tf::Vector3(x_enu, y_enu, 0.0));
     tf::Quaternion q;
     q.setRPY(0, 0, heading);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp  , "odom", "gps"));

}


int main(int argc, char **argv){
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/swiftnav/front/gps_pose", 10, callback);
    ros::spin();

    return 0;
}