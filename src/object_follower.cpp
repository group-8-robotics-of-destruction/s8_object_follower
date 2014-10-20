#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <s8_ip/distPose.h>
#include <s8_common_node/Node.h>

#define NODE_NAME           "s8_object_follower_node"

#define HZ                  10
#define BUFFER_SIZE         1000

#define TOPIC_TWIST         "/s8/twist"
#define TOPIC_DIST_POSE     "/s8_ip/distPose"

#define PARAM_DIST_TRESHOLD_NEAR_NAME           "dist_treshold_near"
#define PARAM_DIST_TRESHOLD_NEAR_DEFAULT        650
#define PARAM_DIST_TRESHOLD_FAR_NAME            "dist_treshold_far"
#define PARAM_DIST_TRESHOLD_FAR_DEFAULT         1000
#define PARAM_DIST_BACK_NAME                    "dist_back"
#define PARAM_DIST_BACK_DEFAULT                 250
#define PARAM_SPEED_NAME                        "speed"
#define PARAM_SPEED_DEFAULT                     0.4

class WallFollower : public s8::Node {
    const int hz;

    ros::Publisher twist_publisher;
    ros::Subscriber dist_pose_subscriber;

    double w;
    double v;

    int dist_treshold_near;
    int dist_treshold_far;
    int dist_back;

    double speed;

public:
    WallFollower(int hz) : hz(hz), v(0.0), w(0.0) {
        add_params();
        print_params();
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, BUFFER_SIZE);
        dist_pose_subscriber = nh.subscribe<s8_ip::distPose>(TOPIC_DIST_POSE, BUFFER_SIZE, &WallFollower::dist_pose_callback, this);
    }

    void update() {
        publish_twist();
    }

private:
    void dist_pose_callback(const s8_ip::distPose::ConstPtr & dist_pose) {
        int pose = dist_pose->pose;
        int dist = dist_pose->dist;
        compute_twist(dist, pose, v, w);
        ROS_INFO("pose: %d dist: %d. Computed v: %lf w: %lf", pose, dist, v, w);
    }

    void publish_twist() {
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        twist.angular.z = w;
        twist_publisher.publish(twist);
        ROS_INFO("v: %lf w: %lf", v, w);
    }

    void compute_twist(double dist, double pose, double & v, double & w) {
        if(dist > dist_treshold_far) {
            v = 0.0;
            w = 0.0;
        }
        else if ( dist < dist_back){
            v = -speed;
            w = 0.0;
        }
        else if (dist > dist_back && dist < dist_treshold_near){
            v = 0.0;
            w = -pose / (dist/5.0);
        }
        else {
            v = speed;
            w = -pose / (dist/5.0);
        }
    ROS_INFO("w: %lf, pose: %lf, distance: %lf", w,pose,dist);
    }

    void add_params() {
        add_param(PARAM_DIST_TRESHOLD_NEAR_NAME, dist_treshold_near, PARAM_DIST_TRESHOLD_NEAR_DEFAULT);
        add_param(PARAM_DIST_TRESHOLD_FAR_NAME, dist_treshold_far, PARAM_DIST_TRESHOLD_FAR_DEFAULT);
        add_param(PARAM_DIST_BACK_NAME, dist_back, PARAM_DIST_BACK_DEFAULT);
        add_param(PARAM_SPEED_NAME, speed, PARAM_SPEED_DEFAULT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    WallFollower wall_follower(HZ);
    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        wall_follower.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
