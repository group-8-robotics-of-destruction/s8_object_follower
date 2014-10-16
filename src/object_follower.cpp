#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <s8_ip/distPose.h>
#include <s8_common_node/Node.h>

#define NODE_NAME           "s8_wall_follower_node"

#define HZ                  10
#define BUFFER_SIZE         1000

#define TOPIC_TWIST         "/s8/twist"
#define TOPIC_DIST_POSE     "/s8/dist_pose"

class WallFollower : public s8::Node {
    const int hz;

    ros::Publisher twist_publisher;
    ros::Subscriber dist_pose_subscriber;

public:
    WallFollower(int hz) : hz(hz) {
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
        ROS_INFO("pose: %d dist: %d", pose, dist);
    }

    void publish_twist() {
        double v = 0.5;
        double w = 0.5;
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        twist.angular.z = w;
        ROS_INFO("v: %lf w: %lf", v, w);
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