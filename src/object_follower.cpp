#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <s8_ip/distPose.h>
#include <s8_common_node/Node.h>

#define NODE_NAME           "s8_wall_follower_node"

#define HZ                  10
#define BUFFER_SIZE         1000

#define TOPIC_TWIST         "/s8/twist"
#define TOPIC_DIST_POSE     "/s8_ip/distPose"

class WallFollower : public s8::Node {
    const int hz;

    ros::Publisher twist_publisher;
    ros::Subscriber dist_pose_subscriber;

	double w;
	double v;

public:
    WallFollower(int hz) : hz(hz), v(0.0), w(0.0) {
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
        w = -pose / 100.0;
        v = 0.3;
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
