/**
 * 6-DOF Initial Pose Interactive Marker for RViz2.
 * Drag the sphere to translate, rotate rings for orientation.
 * Publishes to /initialpose on every change.
 * Also follows /initialpose so it stays at the last known pose.
 */

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <interactive_markers/interactive_marker_server.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class InitialPose3DMarker : public rclcpp::Node
{
public:
  InitialPose3DMarker()
  : Node("initial_pose_3d_marker")
  {
    this->declare_parameter("init_x", 0.0);
    this->declare_parameter("init_y", 0.0);
    this->declare_parameter("init_z", 0.2);

    pose_.position.x = this->get_parameter("init_x").as_double();
    pose_.position.y = this->get_parameter("init_y").as_double();
    pose_.position.z = this->get_parameter("init_z").as_double();
    pose_.orientation.w = 1.0;

    // TransientLocal: late subscribers (e.g. ros2 topic echo) receive last msg
    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(10).transient_local());

    sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          if (msg->header.frame_id != "__marker_internal__") {
            pose_ = msg->pose.pose;
            server_->erase("initial_pose_3d");
            createMarker();
            publishPose();
            RCLCPP_INFO(this->get_logger(),
                "Marker moved to external pose: (%.1f, %.1f, %.1f)",
                pose_.position.x, pose_.position.y, pose_.position.z);
          }
        });

    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "/initial_pose_3d_marker", this);

    createMarker();
    publishPose();

    RCLCPP_INFO(this->get_logger(),
        "3D Initial Pose Marker ready on /initial_pose_3d_marker/update");
  }

private:
  void createMarker()
  {
    auto mkMarker = [](float r, float g, float b, float a, int type,
                       float sx, float sy, float sz) {
      visualization_msgs::msg::Marker m;
      m.type = type;
      m.scale.x = sx; m.scale.y = sy; m.scale.z = sz;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
      return m;
    };

    visualization_msgs::msg::InteractiveMarker im;
    im.header.frame_id = "world";
    im.name = "initial_pose_3d";
    im.description = "6-DOF Pose: drag sphere (move) | rings (rotate)";
    im.pose = pose_;
    im.scale = 2.0;

    // --- visual: cyan sphere + coordinate arrows ---
    {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.always_visible = true;
      ctrl.markers.push_back(mkMarker(0.2f, 0.8f, 1.0f, 0.6f,
          visualization_msgs::msg::Marker::SPHERE, 0.6, 0.6, 0.6));
      // X arrow (red)
      ctrl.markers.push_back(mkMarker(1.0f, 0.2f, 0.2f, 0.8f,
          visualization_msgs::msg::Marker::ARROW, 1.2, 0.08, 0.08));
      im.controls.push_back(ctrl);
    }

    // --- rotate yaw (Z ring, blue) ---
    {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.name = "rotate_yaw";
      ctrl.orientation.w = 1.0; ctrl.orientation.z = 1.0;
      ctrl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      im.controls.push_back(ctrl);
    }

    // --- rotate pitch (Y ring, green) ---
    {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.name = "rotate_pitch";
      ctrl.orientation.w = 1.0; ctrl.orientation.y = 1.0;
      ctrl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      im.controls.push_back(ctrl);
    }

    // --- rotate roll (X ring, red) ---
    {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.name = "rotate_roll";
      ctrl.orientation.w = 1.0; ctrl.orientation.x = 1.0;
      ctrl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      im.controls.push_back(ctrl);
    }

    // --- move XY (horizontal plane) ---
    {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.name = "move_xy";
      ctrl.orientation.w = 1.0;
      ctrl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
      im.controls.push_back(ctrl);
    }

    // --- move Z (vertical axis) ---
    {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.name = "move_z";
      ctrl.orientation.w = 1.0; ctrl.orientation.x = 1.0;
      ctrl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      im.controls.push_back(ctrl);
    }

    server_->insert(im);
    server_->setCallback(
        im.name,
        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & fb) {
          if (fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
            pose_ = fb->pose;
            publishPose();
          }
        },
        visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);

    server_->applyChanges();
  }

  void publishPose()
  {
    auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "__marker_internal__";  // tag so we don't echo
    msg->pose.pose = pose_;
    msg->pose.covariance.fill(0.0);
    msg->pose.covariance[0]  = 0.25;
    msg->pose.covariance[7]  = 0.25;
    msg->pose.covariance[35] = 0.06853891909122467;

    pub_->publish(std::move(msg));

    double r, p, y;
    tf2::Quaternion q(pose_.orientation.x, pose_.orientation.y,
                      pose_.orientation.z, pose_.orientation.w);
    tf2::Matrix3x3(q).getRPY(r, p, y);
    RCLCPP_INFO(this->get_logger(),
        "Pose: (%.1f, %.1f, %.1f) rpy(%.0f, %.0f, %.0f)°",
        pose_.position.x, pose_.position.y, pose_.position.z,
        r * 180.0 / M_PI, p * 180.0 / M_PI, y * 180.0 / M_PI);
  }

  geometry_msgs::msg::Pose pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPose3DMarker>());
  rclcpp::shutdown();
  return 0;
}
