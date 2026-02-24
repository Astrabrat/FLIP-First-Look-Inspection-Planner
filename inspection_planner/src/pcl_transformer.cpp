#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

class PclTransformerNode : public rclcpp::Node {
public:
  PclTransformerNode()
  : Node(
        "pcl_transformer",
        rclcpp::NodeOptions().allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    // Params with sane defaults (keep ROS 1 names)
    target_frame_ = this->declare_or_get_string_("frame_id", "base_link");
    input_topic_ = this->declare_or_get_string_("input_pointcloud", "/points_raw");
    output_topic_ = this->declare_or_get_string_("output_pointcloud", "/points_transformed");
    sensor_type_ = this->declare_or_get_string_("sensor", "lidar");

    filter_min_x_ = this->declare_or_get_double_("filter_min_x", -50.0);
    filter_max_x_ = this->declare_or_get_double_("filter_max_x",  50.0);
    filter_min_y_ = this->declare_or_get_double_("filter_min_y", -50.0);
    filter_max_y_ = this->declare_or_get_double_("filter_max_y",  0.0);
    filter_min_z_ = this->declare_or_get_double_("filter_min_z",  0.0);
    filter_max_z_ = this->declare_or_get_double_("filter_max_z",  5.0);

    voxel_leaf_x_ = this->declare_or_get_double_("voxel_leaf_x", 0.2);
    voxel_leaf_y_ = this->declare_or_get_double_("voxel_leaf_y", 0.2);
    voxel_leaf_z_ = this->declare_or_get_double_("voxel_leaf_z", 0.2);

    // ---- Print parameters ----
    RCLCPP_INFO(this->get_logger(),
        "Filter X: [%.3f, %.3f]", filter_min_x_, filter_max_x_);

    RCLCPP_INFO(this->get_logger(),
        "Filter Y: [%.3f, %.3f]", filter_min_y_, filter_max_y_);

    RCLCPP_INFO(this->get_logger(),
        "Filter Z: [%.3f, %.3f]", filter_min_z_, filter_max_z_);

    RCLCPP_INFO(this->get_logger(),
        "Voxel leaf: [%.3f, %.3f, %.3f]",
        voxel_leaf_x_, voxel_leaf_y_, voxel_leaf_z_);

    auto qos = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos, std::bind(&PclTransformerNode::callback, this, _1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

    RCLCPP_INFO(this->get_logger(),
                "PclTransformerNode up: input=%s output=%s -> %s sensor=%s",
                input_topic_.c_str(), output_topic_.c_str(),
                target_frame_.c_str(), sensor_type_.c_str());
  }

private:
  std::string declare_or_get_string_(const std::string & name, const std::string & def) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<std::string>(name, def);
    }
    return this->get_parameter(name).as_string();
  }

  double declare_or_get_double_(const std::string & name, double def) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<double>(name, def);
    }
    return this->get_parameter(name).as_double();
  }

  static void passThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                          const std::string & field,
                          double min_v,
                          double max_v) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(static_cast<float>(min_v), static_cast<float>(max_v));
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pass.filter(tmp);
    *cloud = std::move(tmp);
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1) Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pc);

    // 2) Downsample + filter (kept identical to ROS 1 logic)
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(pc);

    if (sensor_type_ == "depth_camera" || sensor_type_ == "lidar") {
      if (sensor_type_ == "depth_camera") {
        vox.setLeafSize(static_cast<float>(voxel_leaf_x_),
                        static_cast<float>(voxel_leaf_y_),
                        static_cast<float>(voxel_leaf_z_));
        vox.filter(*pc);
        passThrough(pc, "z", filter_min_z_, filter_max_z_);
      } else {
        passThrough(pc, "z", filter_min_z_, filter_max_z_);
        passThrough(pc, "y", filter_min_y_, filter_max_y_);
        passThrough(pc, "x", filter_min_x_, filter_max_x_);
      }
    }

    // 3) Back to ROS msg (still in source frame)
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*pc, filtered_msg);
    filtered_msg.header = msg->header;  // keep original frame + stamp for TF

    // 4) Transform to target_frame_ using tf2 at the cloud timestamp
    try {
      const rclcpp::Time stamp(msg->header.stamp);

      // Keep the timeout small like ROS 1 version.
      if (!tf_buffer_.canTransform(target_frame_, msg->header.frame_id, stamp,
                                   tf2::durationFromSec(0.1))) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "No transform %s -> %s at cloud time yet.",
            msg->header.frame_id.c_str(), target_frame_.c_str());
        return;
      }

      geometry_msgs::msg::TransformStamped T =
          tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, stamp);

      sensor_msgs::msg::PointCloud2 out;
      tf2::doTransform(filtered_msg, out, T);

      // Keep the original timestamp (better for downstream sync)
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = target_frame_;
      pub_->publish(out);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "TF2 transform failed: %s", ex.what());
      return;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Params
  std::string target_frame_, input_topic_, output_topic_, sensor_type_;
  double filter_min_x_, filter_max_x_, filter_min_y_, filter_max_y_, filter_min_z_, filter_max_z_;
  double voxel_leaf_x_, voxel_leaf_y_, voxel_leaf_z_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PclTransformerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
