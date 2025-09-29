#ifndef FLAME_ROS_FLAME_HPP
#define FLAME_ROS_FLAME_HPP

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <ros_sensor_streams/conversions.h>

#include <flame/utils/image_utils.h>
#include <flame/utils/stats_tracker.h>
#include <flame/utils/load_tracker.h>

#include <flame/flame.h>
#include <flame/params.h>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <flame_ros/utils.hpp>

#include <opencv2/core/eigen.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sophus/se3.hpp>

#include <mrs_lib/attitude_converter.h>

// ========================
#include <memory>
#include <string>

#include <image_transport/image_transport.hpp>
#include <image_transport/camera_subscriber.hpp>

#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

#include <tf2_ros/buffer.h>

#include <rclcpp/time.hpp>

#include <opencv2/calib3d.hpp>


namespace fu = flame::utils;

#define CHECK_INIT \
  if (!is_initialized_) { \
      return; \
  }

void crash_handler(int /*sig*/) {
  FLAME_ASSERT(false);
  return;
}

namespace flame_ros {

class FlameRos : public rclcpp::Node
{
  public:
    FlameRos(const rclcpp::NodeOptions & options);

        /**
    * \brief Constructor.
    *
    * NOTE: Default, no-args constructor must exist.
    */
    FlameRos() = default;

    virtual ~FlameRos() {
      if (thread_.joinable()) {
        thread_.join();
      }

      return;
    }

    FlameRos(const FlameRos& rhs) = delete;
    FlameRos& operator=(const FlameRos& rhs) = delete;

    FlameRos(FlameRos&& rhs) = delete;
    FlameRos& operator=(FlameRos&& rhs) = delete;

    bool is_initialized(){return is_initialized_;}

  private:
    rclcpp::TimerBase::SharedPtr timer_initialization_;
    bool is_initialized_;
    void timerInitialization();
    void callback(const std::shared_ptr<const sensor_msgs::msg::Image>& rgb_msg, const std::shared_ptr<const sensor_msgs::msg::CameraInfo>& info);
    void poseframeCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void append_odom_to_path(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void processFrame(const uint32_t img_id, const std::string& cam_frame_id, const double time, const Sophus::SE3f& pose, const cv::Mat3b& rgb);
    void main();

    // // Convenience alias.
    //using Frame = ros_sensor_streams::TrackedImageStream::Frame;

    #ifdef FLAME_WITH_FLA
      enum Status {
        GOOD = 0,
        ALARM_TIMEOUT = 2,
        FAIL_TIMEOUT = 3,
      };
    #endif

    //std::thread thread_;
    //fu::LoadTracker load_;

    std::thread thread_;

    // Keeps track of stats and load.
    fu::StatsTracker stats_;
    fu::LoadTracker load_;

    // Number of images processed.
    int num_imgs_;

    // tf stuff.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    // input params.
    std::string camera_frame_id_; // Frame id of the camera in optical coordinates.
    std::string camera_world_frame_id_; // Frame id of the world in camera optical coordinates.
    int subsample_factor_; // Process one out of this many images.
    int poseframe_subsample_factor_; // Create a poseframe every this number of images.
    int resize_factor_; // Factor to resize image. resize_factor_ = 2 will downsample by 2 in each dimension.

    // Use an external calibration instead of what's in the camera_info message.
    bool use_external_cal_;

    // Input stream object.
    //std::shared_ptr<ros_sensor_streams::TrackedImageStream> input_;
    Eigen::Matrix3f Kinv_;

    // PoseFrame stuff.
    std::string poseframe_child_frame_id_; // Frame inside pf messages.
    bool pfs_inited_; // False until first poseframe message received with > 2 pfs.
    uint32_t first_pf_id_; // ID of first poseframe.
    bool use_poseframe_updates_;
    ////ros::Subscriber poseframe_sub_;
    //mrs_lib::SubscriberHandler<nav_msgs::msg::Path> poseframe_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr poseframe_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    nav_msgs::msg::Path::SharedPtr odom_path;

    // Stuff for checking angular rates.
    float max_angular_rate_;
    double prev_time_;
    Sophus::SE3f prev_pose_;

    // Depth sensor.
    flame::Params params_;
    std::shared_ptr<flame::Flame> sensor_;

    std::vector<uint32_t> poses_ids_;

    // Publishes mesh.
    bool publish_mesh_;
    ////ros::Publisher mesh_pub_;
    mrs_lib::PublisherHandler<pcl_msgs::msg::PolygonMesh> mesh_pub_;

    // Publishes depthmap.
    cv::Mat1f idepthmap_;
    bool publish_idepthmap_;
    bool publish_depthmap_;
    bool publish_features_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraPublisher idepth_pub_;
    image_transport::CameraPublisher depth_pub_;
    image_transport::CameraPublisher features_pub_;

    // Publish pointcloud.
    bool publish_cloud_;
    ////ros::Publisher cloud_pub_;
    mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> cloud_pub_;

    // Publishes statistics.
    bool publish_stats_;
    mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameStats> stats_pub_;
    mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameNodeletStats> nodelet_stats_pub_;
    int load_integration_factor_;

    // Publishes debug images.
    image_transport::Publisher debug_wireframe_pub_;
    image_transport::Publisher debug_features_pub_;
    image_transport::Publisher debug_detections_pub_;
    image_transport::Publisher debug_matches_pub_;
    image_transport::Publisher debug_normals_pub_;
    image_transport::Publisher debug_idepthmap_pub_;

  #ifdef FLAME_WITH_FLA
    uint8_t node_id_;
    double heart_beat_dt_;
    double alarm_timeout_;
    double fail_timeout_;
    ros::Timer heart_beat_;
    ros::Publisher heart_beat_pub_;
    double last_update_sec_;
  #endif

    // Messages in the ROS2 does not have "seq" field in the header.
    // We have to replace it by the counter in the subscriber.
    unsigned long int pose_frame_id;

    bool inited_;
    bool undistort_; // Whether to undistort images.

    std::string live_frame_id_;
    int width_;
    int height_;
    Eigen::Matrix3f K_; // Camera intrinsics.
    Eigen::VectorXf D_; // Distortion params: k1, k2, p1, p2, k3.

    //std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    //tf2_ros::Buffer tf_buffer_;

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::CameraSubscriber cam_sub_;

    //ThreadSafeQueue<Frame> queue_;

    long unsigned int frame_counter;
};

} // namespace flame_ros

#endif
