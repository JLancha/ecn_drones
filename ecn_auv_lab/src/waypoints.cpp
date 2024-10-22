#include "waypoints.h"

#include <rclcpp/rclcpp.hpp>
#include <exception>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <nav_msgs/msg/odometry.h>


using namespace geometry_msgs::msg;
using namespace std::chrono_literals;

class Control : public rclcpp::Node
{
public:
  Control() : Node("waypoints")
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    // load waypoints and thresholds
    Waypoint::load(waypoints, position_thr, orientation_thr);

    pose_cmd.header.frame_id = "world";
    pub = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 1);

    static auto timer{create_wall_timer(10ms, [&]()
      {
        if(!buffer.canTransform("world", "bluerov2/base_link", tf2::TimePointZero))
          return;
        const auto transform{buffer.lookupTransform("world", "bluerov2/base_link", tf2::TimePointZero)};
        trackWaypoint(transform.transform);
      })};

  }

private:

  std::vector<Waypoint> waypoints;
  double position_thr, orientation_thr;

  tf2_ros::Buffer buffer{get_clock()};
  tf2_ros::TransformListener tl{buffer};

  PoseStamped pose_cmd;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub;



  void trackWaypoint(const Transform &pose)
  {
    static auto cur_wp{0};

    //get auv position and yaw:
    float auv_x = pose.translation.x;
    float auv_y = pose.translation.y;
    float auv_z = pose.translation.z;

    float auv_rz = pose.rotation.z;
    float auv_rw = pose.rotation.w;
    float auv_theta = 2*std::atan2(auv_rz,auv_rw);

    // TODO update cur_wp to cycle through the waypoints when the current one is reached
    float distance = std::pow(auv_x - waypoints[cur_wp].x, 2);
    distance += std::pow(auv_y - waypoints[cur_wp].y, 2);
    distance += std::pow(auv_z - waypoints[cur_wp].z, 2);
    distance = std::sqrt(distance);

    float orientation_err = auv_theta - waypoints[cur_wp].theta;
    //clip the error between +-PI.
    orientation_err = fmod(orientation_err + M_PI, 2*M_PI);

    if(orientation_err < 0){
        orientation_err += 2*M_PI;
    }
    orientation_err -= M_PI;

    if(distance < position_thr && orientation_err < orientation_thr){
        if(cur_wp == waypoints.size())
        {
            cur_wp = 0;
        }else{
            cur_wp += 1;
        }
    }

    waypoints[cur_wp].write(pose_cmd);
    pose_cmd.header.stamp = get_clock()->now();
    pub->publish(pose_cmd);
  }
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
