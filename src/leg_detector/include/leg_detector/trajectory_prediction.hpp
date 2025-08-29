#include <rclcpp/rclcpp.hpp>
#include <leg_detector_msgs/msg/trajectory.hpp>
#include <leg_detector_msgs/msg/trajectory_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <utility>
#include <vector>

class Line
{
public:
  double getY(double x);
  double getA();
  double getB();
  double getC();
  void addPoints(std::vector<double> x, std::vector<double> y);

private:
  double a;
  double b;
  double c;
};

class TrajectoryPredictionNode : public rclcpp::Node
{
public:
  TrajectoryPredictionNode();
private:
  void trajectoryArrayCallback(const leg_detector_msgs::msg::TrajectoryArray::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &params);
  rclcpp::Subscription<leg_detector_msgs::msg::TrajectoryArray>::SharedPtr trajectoryArraySubscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predictedTrajectoryPublisher_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string trajectory_array_topic_;
  std::string trajectory_visualization_topic_;
};
