#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PclTransformerNode {
public:
  PclTransformerNode() : tf_buffer_(), tf_listener_(tf_buffer_) {
    ros::NodeHandle pnh("~");

    // Params with sane defaults
    pnh.param<std::string>("frame_id", target_frame_, "base_link");
    pnh.param<std::string>("input_pointcloud", input_topic_, "/points_raw");
    pnh.param<std::string>("output_pointcloud", output_topic_, "/points_transformed");
    pnh.param<std::string>("sensor", sensor_type_, "lidar");

    pnh.param("filter_min_x", filter_min_x_, -50.0);
    pnh.param("filter_max_x", filter_max_x_,  50.0);
    pnh.param("filter_min_y", filter_min_y_, -50.0);
    pnh.param("filter_max_y", filter_max_y_,  50.0);
    pnh.param("filter_min_z", filter_min_z_, -3.0);
    pnh.param("filter_max_z", filter_max_z_,  5.0);

    pnh.param("voxel_leaf_x", voxel_leaf_x_, 0.2);
    pnh.param("voxel_leaf_y", voxel_leaf_y_, 0.2);
    pnh.param("voxel_leaf_z", voxel_leaf_z_, 0.2);

    sub_ = nh_.subscribe(input_topic_, 1, &PclTransformerNode::callback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

    ROS_INFO_STREAM("PclTransformerNode up: input=" << input_topic_
                    << " output=" << output_topic_
                    << " -> " << target_frame_
                    << " sensor=" << sensor_type_);
  }

private:
  void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1) Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pc);

    // 2) Downsample
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(pc);
    // 3) PassThrough(s)
    if (sensor_type_ == "depth_camera" || sensor_type_ == "lidar") {
      // Apply Z for depth cams; X/Y for lidars; feel free to adapt
      if (sensor_type_ == "depth_camera") {
        vox.setLeafSize(static_cast<float>(voxel_leaf_x_),
                      static_cast<float>(voxel_leaf_y_),
                      static_cast<float>(voxel_leaf_z_));
        vox.filter(*pc);
        // passThrough(pc, "y", filter_min_y_, filter_max_y_);
        passThrough(pc, "z", filter_min_z_, filter_max_z_);
        // passThrough(pc, "x", filter_min_z_, filter_max_z_);
      } else {
        passThrough(pc, "z", filter_min_z_, filter_max_z_);
        passThrough(pc, "y", filter_min_y_, filter_max_y_);
        passThrough(pc, "x", filter_min_x_, filter_max_x_);

      }
    }

    // 4) Back to ROS msg (still in source frame)
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*pc, filtered_msg);
    filtered_msg.header = msg->header; // keep original frame + stamp for TF

    // 5) Transform to target_frame_ using tf2 at the cloud timestamp
    try {
      // Wait up to 0.5s for the transform at the message time
    //   tf2::TimePoint tf2_time = tf2_ros::fromMsg(msg->header.stamp);
      if (!tf_buffer_.canTransform(target_frame_, msg->header.frame_id, msg->header.stamp,
                                   ros::Duration(0.05))) {
        ROS_WARN_STREAM_THROTTLE(1.0, "No transform " << msg->header.frame_id
                                << " -> " << target_frame_ << " at time "
                                << msg->header.stamp << " yet.");
        return;
      }

      geometry_msgs::TransformStamped T =
          tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);

      sensor_msgs::PointCloud2 out;
      tf2::doTransform(filtered_msg, out, T);
      // Keep the original timestamp (better for downstream sync)
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = target_frame_;
      pub_.publish(out);
      ros::Duration(0.01).sleep();
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_STREAM_THROTTLE(1.0, "TF2 transform failed: " << ex.what());
      return;
    }
  }

  static void passThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const std::string& field, double min_v, double max_v) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(static_cast<float>(min_v), static_cast<float>(max_v));
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pass.filter(tmp);
    *cloud = std::move(tmp);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Params
  std::string target_frame_, input_topic_, output_topic_, sensor_type_;
  double filter_min_x_, filter_max_x_, filter_min_y_, filter_max_y_, filter_min_z_, filter_max_z_;
  double voxel_leaf_x_, voxel_leaf_y_, voxel_leaf_z_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_transformer");
  PclTransformerNode node;
  ros::spin();
  return 0;
}
