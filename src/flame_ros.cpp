/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/node.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/mutex.h>

#include <ros_sensor_streams/conversions.h>

#include <flame/flame.h>
#include <flame/params.h>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/core/eigen.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sophus/se3.hpp>

#include <mrs_lib/attitude_converter.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/camera_subscriber.hpp>

#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

#include <tf2_ros/buffer.h>

#include <rclcpp/time.hpp>

#include <opencv2/calib3d.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>

#include <flame/utils/image_utils.h>
#include <flame/utils/visualization.h>
#include <flame/utils/triangulator.h>
#include <flame/utils/stats_tracker.h>
#include <flame/utils/load_tracker.h>

#include <flame_ros_msgs/msg/flame_nodelet_stats.hpp>
#include <flame_ros_msgs/msg/flame_stats.hpp>

#include <string>
#include <limits>
#include <memory>
#include <string>

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

#include <image_transport/camera_publisher.hpp>

#include <mrs_lib/publisher_handler.h>

#include <flame_ros_msgs/msg/flame_nodelet_stats.hpp>
#include <flame_ros_msgs/msg/flame_stats.hpp>

//}

/* defines //{ */

#define NODE_NAME "flame"

#define PCL_NO_PRECOMPILE

//}

namespace fu = flame::utils;

/* structs //{ */

namespace flame_ros
{

/**
 * @breif Struct to hold mesh vertex data.
 */
struct PointNormalUV
{
  PCL_ADD_POINT4D
  PCL_ADD_NORMAL4D
  float u; // Texture coordinates.
  float v;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace flame_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(flame_ros::PointNormalUV, (float, x, x)(float, y, y)(float, x, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(
                                                                float, normal_z, normal_z)(float, u, u)(float, v, v))

//}

namespace flame_ros
{

/* class FlameRos //{ */

void crash_handler(int /*sig*/) {
  FLAME_ASSERT(false);
  return;
}

class FlameRos : public mrs_lib::Node {
public:
  FlameRos(const rclcpp::NodeOptions &options);

private:
  bool is_initialized_ = false;
  void initialize();
  void callbackCamera(const std::shared_ptr<const sensor_msgs::msg::Image> &rgb_msg, const std::shared_ptr<const sensor_msgs::msg::CameraInfo> &info);
  void processFrame(const uint32_t img_id, const std::string &cam_frame_id, const double time, const Sophus::SE3f &pose, const cv::Mat3b &rgb);

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  // // Convenience alias.
  // using Frame = ros_sensor_streams::TrackedImageStream::Frame;

  // Keeps track of stats and load.
  fu::StatsTracker stats_;
  fu::LoadTracker  load_;

  // Number of images processed.
  int num_imgs_;

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // input params.
  std::string _uav_name_;
  std::string _world_frame_;
  std::string _body_frame_;
  std::string camera_frame_;
  std::string _camera_frame_;
  std::string _path_frame_;

  double resize_factor_;

  // Input stream object.
  // std::shared_ptr<ros_sensor_streams::TrackedImageStream> input_;
  Eigen::Matrix3f Kinv_;

  // PoseFrame stuff.
  std::atomic<bool> pfs_inited_ = false; // False until first poseframe message received with > 2 pfs.

  ////ros::Subscriber poseframe_sub_;
  // mrs_lib::SubscriberHandler<nav_msgs::msg::Path> poseframe_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     poseframe_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Stuff for checking angular rates.
  float        max_angular_rate_;
  double       prev_time_;
  Sophus::SE3f prev_pose_;

  // Depth sensor.
  flame::Params                 params_;
  std::shared_ptr<flame::Flame> sensor_;

  double  _undistort_balance_;
  cv::Mat _custom_fisheye_undistorted_K_;
  cv::Mat map1, map2;

  std::vector<uint32_t> poses_ids_;

  // Publishes mesh.
  bool publish_mesh_;
  ////ros::Publisher mesh_pub_;
  mrs_lib::PublisherHandler<pcl_msgs::msg::PolygonMesh> mesh_pub_;

  // Publishes depthmap.
  cv::Mat1f                                        idepthmap_;
  bool                                             publish_idepthmap_;
  bool                                             publish_depthmap_;
  bool                                             publish_features_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher                 idepth_pub_;
  image_transport::CameraPublisher                 depth_pub_;
  image_transport::CameraPublisher                 features_pub_;

  // Publish pointcloud.
  bool publish_cloud_;
  ////ros::Publisher cloud_pub_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> cloud_pub_;

  // Publishes statistics.
  bool                                                              publish_stats_;
  mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameStats>        stats_pub_;
  mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameNodeletStats> nodelet_stats_pub_;
  int                                                               load_integration_factor_;

  bool publish_undistort_;
  void publishUndistortedImage(const cv::Mat3b &rgb, const std_msgs::msg::Header &orig_header);

  // Publishes debug images.
  image_transport::CameraPublisher debug_undistort_;
  image_transport::Publisher       debug_wireframe_pub_;
  image_transport::Publisher       debug_features_pub_;
  image_transport::Publisher       debug_detections_pub_;
  image_transport::Publisher       debug_matches_pub_;
  image_transport::Publisher       debug_normals_pub_;
  image_transport::Publisher       debug_idepthmap_pub_;

  bool inited_ = false;

  bool _undistort_enabled_; // Whether to undistort images.
  bool _undistort_fisheye_enabled_;
  int  _custom_fisheye_expected_width_;
  int  _custom_fisheye_expected_height_;

  std::string     live_frame_id_;
  int             width_  = 0;
  int             height_ = 0;
  Eigen::Matrix3f K_; // Camera intrinsics.
  Eigen::Matrix3f K_resized_; // Camera intrinsics.
  Eigen::VectorXf D_; // Distortion params: k1, k2, p1, p2, k3.

  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::CameraSubscriber                cam_sub_;

  // ThreadSafeQueue<Frame> queue_;j

  long unsigned int frame_counter = 0;

  // | --------------- dynamic reconfigure server --------------- |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;

  struct Params_t
  {
    int    max_pose_frames;
    int    poseframe_mod_factor;
    double cloud_decimation;
  };

  Params_t   drs_params_;
  std::mutex mutex_drs_params_;

  /**
   * @brief Publish stats message for FlameNodelet.
   */
  void publishFlameNodeletStats(mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameNodeletStats> &pub, int img_id, double time,
                                const std::unordered_map<std::string, double> &stats, const std::unordered_map<std::string, double> &timings);

  /**
   * @brief Publish stats message for Flame.
   */
  void publishFlameStats(mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameStats> &pub, int img_id, double time,
                         const std::unordered_map<std::string, double> &stats, const std::unordered_map<std::string, double> &timings);

  /**
   * @brief Publish mesh.
   */
  void publishDepthMesh(mrs_lib::PublisherHandler<pcl_msgs::msg::PolygonMesh> &mesh_pub, const std::string &frame_id, double time, const Eigen::Matrix3f &Kinv,
                        const std::vector<cv::Point2f> &vertices, const std::vector<float> &idepths, const std::vector<Eigen::Vector3f> &normals,
                        const std::vector<flame::Triangle> &triangles, const std::vector<bool> &tri_validity, const cv::Mat3b &rgb);

  /**
   * @brief Publish depthmap.
   */
  void publishDepthMap(const image_transport::CameraPublisher &pub, const std::string &frame_id, double time, const Eigen::Matrix3f &K,
                       const cv::Mat1f &depth_est);

  /**
   * @brief Publish point cloud.
   */
  void publishPointCloud(mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> &pub, const std::string &frame_id, double time, const Eigen::Matrix3f &K,
                         const cv::Mat1f &depth_est, float min_depth = 0.0f, float max_depth = std::numeric_limits<float>::max());

  /**
   * @brief Compute confusion matrix using ground truth depths.
   */
  void getDepthConfusionMatrix(const cv::Mat1f &idepths, const cv::Mat1f &depth, cv::Mat1f *idepth_error, float *total_error, int *true_pos, int *true_neg,
                               int *false_pos, int *false_neg);
};

//}

/* constructor //{ */

FlameRos::FlameRos(const rclcpp::NodeOptions &options) : mrs_lib::Node(NODE_NAME, options), live_frame_id_(), D_(5), cam_sub_() {
  initialize();
}

//}

/* initialize //{ */

void FlameRos::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  /* std::signal(SIGSEGV, crash_handler); */
  /* std::signal(SIGILL, crash_handler); */
  /* std::signal(SIGABRT, crash_handler); */
  /* std::signal(SIGFPE, crash_handler); */

  image_transport_ = std::make_unique<image_transport::ImageTransport>(this_node_ptr());

  mrs_lib::ParamLoader param_loader(this_node_ptr(), NODE_NAME);

  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(this_node_ptr(), mutex_drs_params_);

  std::string calibration_file;
  param_loader.loadParam("calibration_file", calibration_file);

  if (calibration_file != "") {
    param_loader.addYamlFile(calibration_file);
  }

  std::string custom_config;
  param_loader.loadParam("custom_config", custom_config);

  if (custom_config != "") {
    param_loader.addYamlFile(custom_config);
  }

  if (!param_loader.addYamlFileFromParam("default_config")) {
    RCLCPP_ERROR(node_->get_logger(), "could not load params from default_config");
    rclcpp::shutdown();
    exit(1);
  }

  dynparam_mgr_->get_param_provider().copyYamls(param_loader.getParamProvider());

  load_ = std::move(fu::LoadTracker(getpid()));

  num_imgs_ = 0;

  /*==================== Input Params ====================*/

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("path_frame", _path_frame_);
  param_loader.loadParam("camera_frame", _camera_frame_);
  param_loader.loadParam("world_frame", _world_frame_);
  param_loader.loadParam("body_frame", _body_frame_);
  param_loader.loadParam("input/resize_factor", resize_factor_);

  param_loader.loadParam("input/undistort/enabled", _undistort_enabled_);
  param_loader.loadParam("input/undistort/custom_fisheye_model/enabled", _undistort_fisheye_enabled_);

  if (_undistort_enabled_ && _undistort_fisheye_enabled_) {

    std::string         distortion_model;
    std::vector<double> distortion_coeffs;
    std::vector<double> intrinsics;
    std::vector<double> resolution;

    param_loader.loadParam("cam0/distortion_model", distortion_model);

    if (distortion_model != "equidistant") {
      RCLCPP_ERROR(node_->get_logger(), "the custom camera distortion model needs to be 'equidistant'");
      rclcpp::shutdown();
      exit(1);
    }

    param_loader.loadParam("cam0/distortion_coeffs", distortion_coeffs);

    if (distortion_coeffs.size() != 4) {
      RCLCPP_ERROR(node_->get_logger(), "the custom camera distortion model needs to have 4 parameters");
      rclcpp::shutdown();
      exit(1);
    }

    param_loader.loadParam("cam0/intrinsics", intrinsics);

    if (intrinsics.size() != 4) {
      RCLCPP_ERROR(node_->get_logger(), "the custom camera intrinsics needs to have 4 parameters");
      rclcpp::shutdown();
      exit(1);
    }

    param_loader.loadParam("cam0/resolution", resolution);

    if (resolution.size() != 2) {
      RCLCPP_ERROR(node_->get_logger(), "the custom camera resolution needs to have 2 parameters");
      rclcpp::shutdown();
      exit(1);
    }

    param_loader.loadParam("input/undistort/custom_fisheye_model/balance", _undistort_balance_);

    cv::Mat K;

    K = (cv::Mat_<double>(3, 3) << intrinsics.at(0), 0.0, intrinsics.at(2), 0.0, intrinsics.at(1), intrinsics.at(3), 0.0, 0.0, 1.0);

    RCLCPP_INFO_STREAM(node_->get_logger(), "  original_K = " << K);

    cv::Mat D;

    D = (cv::Mat_<double>(4, 1) << distortion_coeffs.at(0), distortion_coeffs.at(1), distortion_coeffs.at(2), distortion_coeffs.at(3));

    RCLCPP_INFO_STREAM(node_->get_logger(), "  D = " << D);

    _custom_fisheye_expected_width_  = int(resolution.at(0));
    _custom_fisheye_expected_height_ = int(resolution.at(1));

    // prepare the undistort maps

    cv::Size img_size = {_custom_fisheye_expected_width_, _custom_fisheye_expected_height_};

    RCLCPP_INFO_STREAM(node_->get_logger(), "  img_size = " << img_size);

    // Estimate new camera matrix for the undistorted view
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, cv::Mat::zeros(4, 1, CV_64F), img_size, cv::Mat::eye(3, 3, CV_64F),
                                                            _custom_fisheye_undistorted_K_, _undistort_balance_);

    // Create lookup tables for remapping
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, cv::DataType<double>::type), _custom_fisheye_undistorted_K_, img_size, CV_16SC2, map1, map2);

    RCLCPP_INFO(node_->get_logger(), "custom fisheye calibration loaded:");
    RCLCPP_INFO_STREAM(node_->get_logger(), "  undistorted_K = " << _custom_fisheye_undistorted_K_);
  }

  dynparam_mgr_->register_param("input/max_poseframe_length", &drs_params_.max_pose_frames);
  dynparam_mgr_->register_param("input/poseframe_mod_factor", &drs_params_.poseframe_mod_factor);

  dynparam_mgr_->register_param("output/cloud_decimation", &drs_params_.cloud_decimation);

  /*==================== Output Params ====================*/

  param_loader.loadParam("output/quiet", params_.debug_quiet);
  param_loader.loadParam("output/mesh", publish_mesh_);
  param_loader.loadParam("output/idepthmap", publish_idepthmap_);
  param_loader.loadParam("output/depthmap", publish_depthmap_);
  param_loader.loadParam("output/cloud", publish_cloud_);
  param_loader.loadParam("output/features", publish_features_);
  param_loader.loadParam("output/stats", publish_stats_);
  param_loader.loadParam("output/undistort", publish_undistort_);
  param_loader.loadParam("output/load_integration_factor", load_integration_factor_);
  param_loader.loadParam("output/scene_color_scale", params_.scene_color_scale);
  param_loader.loadParam("output/filter_oblique_triangles", params_.do_oblique_triangle_filter);

  double oblique_normal_thresh;
  param_loader.loadParam("output/oblique_normal_thresh", oblique_normal_thresh);
  params_.oblique_normal_thresh = oblique_normal_thresh;

  param_loader.loadParam("output/oblique_idepth_diff_factor", params_.oblique_idepth_diff_factor);
  param_loader.loadParam("output/oblique_idepth_diff_abs", params_.oblique_idepth_diff_abs);
  param_loader.loadParam("output/filter_long_edges", params_.do_edge_length_filter);

  double edge_length_thresh;
  param_loader.loadParam("output/edge_length_thresh", edge_length_thresh);
  params_.edge_length_thresh = edge_length_thresh;

  param_loader.loadParam("output/filter_triangles_by_idepth", params_.do_idepth_triangle_filter);

  double min_triangle_idepth;
  param_loader.loadParam("output/min_triangle_idepth", min_triangle_idepth);
  params_.min_triangle_idepth = min_triangle_idepth;

  param_loader.loadParam("output/max_angular_rate", max_angular_rate_);

  /*==================== Debug Params ====================*/
  param_loader.loadParam("debug/wireframe", params_.debug_draw_wireframe);
  param_loader.loadParam("debug/features", params_.debug_draw_features);
  param_loader.loadParam("debug/detections", params_.debug_draw_detections);
  param_loader.loadParam("debug/matches", params_.debug_draw_matches);
  param_loader.loadParam("debug/normals", params_.debug_draw_normals);
  param_loader.loadParam("debug/idepthmap", params_.debug_draw_idepthmap);
  param_loader.loadParam("debug/text_overlay", params_.debug_draw_text_overlay);
  param_loader.loadParam("debug/flip_images", params_.debug_flip_images);

  /*==================== Threading Params ====================*/
  param_loader.loadParam("threading/openmp/num_threads", params_.omp_num_threads);
  param_loader.loadParam("threading/openmp/chunk_size", params_.omp_chunk_size);

  /*==================== Feature Params ====================*/
  param_loader.loadParam("features/do_letterbox", params_.do_letterbox);
  param_loader.loadParam("features/detection/min_grad_mag", params_.min_grad_mag);
  params_.fparams.min_grad_mag = params_.min_grad_mag;

  double min_error;
  param_loader.loadParam("features/detection/min_error", min_error);
  params_.min_error = min_error;

  param_loader.loadParam("features/detection/win_size", params_.detection_win_size);

  int win_size;
  param_loader.loadParam("features/tracking/win_size", win_size);
  params_.zparams.win_size = win_size;
  params_.fparams.win_size = win_size;

  param_loader.loadParam("features/tracking/max_dropouts", params_.max_dropouts);

  double epipolar_line_var;
  param_loader.loadParam("features/tracking/epipolar_line_var", epipolar_line_var);
  params_.zparams.epipolar_line_var = epipolar_line_var;

  /*==================== Regularization Params ====================*/
  param_loader.loadParam("regularization/do_nltgv2", params_.do_nltgv2);
  param_loader.loadParam("regularization/nltgv2/adaptive_data_weights", params_.adaptive_data_weights);
  param_loader.loadParam("regularization/nltgv2/rescale_data", params_.rescale_data);
  param_loader.loadParam("regularization/nltgv2/init_with_prediction", params_.init_with_prediction);
  param_loader.loadParam("regularization/nltgv2/idepth_var_max", params_.idepth_var_max_graph);
  param_loader.loadParam("regularization/nltgv2/data_factor", params_.rparams.data_factor);
  param_loader.loadParam("regularization/nltgv2/step_x", params_.rparams.step_x);
  param_loader.loadParam("regularization/nltgv2/step_q", params_.rparams.step_q);
  param_loader.loadParam("regularization/nltgv2/theta", params_.rparams.theta);
  param_loader.loadParam("regularization/nltgv2/min_height", params_.min_height);
  param_loader.loadParam("regularization/nltgv2/max_height", params_.max_height);
  param_loader.loadParam("regularization/nltgv2/check_sticky_obstacles", params_.check_sticky_obstacles);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load params");
    rclcpp::shutdown();
    exit(1);
  }

  // Image resizing not supported for non-FLA.
  /* FLAME_ASSERT(resize_factor_ == 1); */

  // Subscribe to topics.
  it_ = std::make_shared<image_transport::ImageTransport>(this_node_ptr());
  image_transport_.reset(new image_transport::ImageTransport(this_node_ptr()));

  cam_sub_ = image_transport_->subscribeCamera(std::string("~/image_in"), 1,
                                               [this](const sensor_msgs::msg::Image::ConstSharedPtr      &img,
                                                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) { this->callbackCamera(img, info); });

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(this_node_ptr());
  transformer_->retryLookupNewest(true);

  // Set up publishers. For some reason this appears to take a while.
  if (!params_.debug_quiet)
    RCLCPP_INFO(node_->get_logger(), "FlameRos: Setting up publishers...\n");

  if (publish_idepthmap_) {
    idepth_pub_ = it_->advertiseCamera("~/idepth_registered/image_rect_out", 5);
  }
  if (publish_depthmap_) {
    depth_pub_ = it_->advertiseCamera("~/depth_registered/image_rect_out", 5);
  }
  if (publish_features_) {
    features_pub_ = it_->advertiseCamera("~/depth_registered_raw/image_rect_out", 5);
  }
  if (publish_mesh_) {
    mesh_pub_ = mrs_lib::PublisherHandler<pcl_msgs::msg::PolygonMesh>(this_node_ptr(), "~/mesh_out");
  }
  if (publish_cloud_) {
    cloud_pub_ = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(this_node_ptr(), "~/cloud_out");
  }
  if (publish_stats_) {
    stats_pub_         = mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameStats>(this_node_ptr(), "~/stats_out");
    nodelet_stats_pub_ = mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameNodeletStats>(this_node_ptr(), "~/nodelet_stats_out");
  }
  if (publish_undistort_) {
    debug_undistort_ = it_->advertiseCamera("~/debug/undistorted/image_raw", 1);
  }

  if (params_.debug_draw_wireframe) {
    debug_wireframe_pub_ = it_->advertise("~/debug/wireframe", 1);
  }
  if (params_.debug_draw_features) {
    debug_features_pub_ = it_->advertise("~/debug/features", 1);
  }
  if (params_.debug_draw_detections) {
    debug_detections_pub_ = it_->advertise("~/debug/detections", 1);
  }
  if (params_.debug_draw_matches) {
    debug_matches_pub_ = it_->advertise("~/debug/matches", 1);
  }
  if (params_.debug_draw_normals) {
    debug_normals_pub_ = it_->advertise("~/debug/normals", 1);
  }
  if (params_.debug_draw_idepthmap) {
    debug_idepthmap_pub_ = it_->advertise("~/debug/idepthmap", 1);
  }

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* processFrame() //{ */

void FlameRos::processFrame(const uint32_t img_id, const std::string &cam_frame_id, const double time, const Sophus::SE3f &pose, const cv::Mat3b &rgb) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  stats_.tick("process_frame");

  /*==================== Process image ====================*/
  // Convert to grayscale.
  cv::Mat1b img_gray;
  cv::cvtColor(rgb, img_gray, cv::COLOR_RGB2GRAY);

  bool is_poseframe = (img_id % drs_params.poseframe_mod_factor) == 0;

  std::string msg;

  std::vector<uint32_t>     ids   = {img_id};
  std::vector<Sophus::SE3f> poses = {pose};

  sensor_->updatePoseFramePoses(ids, poses);

  if (img_id < 5) {
    RCLCPP_INFO(node_->get_logger(), "waiting for >= 5 images");
    return;
  }

  bool update_success = sensor_->update(time, img_id, pose, img_gray, is_poseframe, msg = msg);

  if (!update_success) {

    stats_.tock("process_frame");

    if (!params_.debug_quiet) {
      RCLCPP_WARN(node_->get_logger(), "Unsuccessful update. Reason: %s\n", msg.c_str());
    }

    return;
  }

  // | -------------------- update the poses -------------------- |

  poses_ids_.insert(poses_ids_.begin(), img_id);

  if (int(poses_ids_.size()) > drs_params.max_pose_frames) {

    poses_ids_.pop_back();

    sensor_->prunePoseFrames(poses_ids_);
  }

  // | ------------------- custom pruning ends ------------------ |

  if (max_angular_rate_ > 0.0f) {

    // Check angle difference between last and current pose. If we're rotating,
    // we shouldn't publish output since it's probably too noisy.
    Eigen::Quaternionf q_delta     = pose.unit_quaternion() * prev_pose_.unit_quaternion().inverse();
    float              angle_delta = fu::fast_abs(Eigen::AngleAxisf(q_delta).angle());
    float              angle_rate  = angle_delta / (time - prev_time_);

    prev_time_ = time;
    prev_pose_ = pose;

    if (angle_rate * 180.0f / M_PI > max_angular_rate_) {
      // Angular rate is too high.
      if (!params_.debug_quiet)
        RCLCPP_ERROR(node_->get_logger(), "Angle Delta = %.3f, rate = %f.3\n", angle_delta * 180.0f / M_PI, angle_rate * 180.0f / M_PI);
      return;
    }
  }

  /*==================== Publish output ====================*/
  stats_.tick("publishing");

  if (publish_mesh_) {

    // Get current mesh.
    std::vector<cv::Point2f>     vtx;
    std::vector<float>           idepths;
    std::vector<Eigen::Vector3f> normals;
    std::vector<flame::Triangle> triangles;
    std::vector<flame::Edge>     edges;
    std::vector<bool>            tri_validity;

    sensor_->getInverseDepthMesh(&vtx, &idepths, &normals, &triangles, &tri_validity, &edges);

    publishDepthMesh(mesh_pub_, cam_frame_id, time, Kinv_, vtx, idepths, normals, triangles, tri_validity, rgb);
  }

  if (publish_idepthmap_ || publish_depthmap_ || publish_cloud_) {

    cv::Mat1f idepthmap;
    sensor_->getFilteredInverseDepthMap(&idepthmap);

    if (publish_idepthmap_) {
      // Publish full idepthmap.
      publishDepthMap(idepth_pub_, cam_frame_id, time, K_resized_, sensor_->getInverseDepthMap());
    }

    // Convert to depths.
    cv::Mat1f depth_est(idepthmap.rows, idepthmap.cols, std::numeric_limits<float>::quiet_NaN());
#pragma omp parallel for collapse(2) num_threads(params_.omp_num_threads) schedule(dynamic, params_.omp_chunk_size) // NOLINT
    for (int ii = 0; ii < depth_est.rows; ++ii) {
      for (int jj = 0; jj < depth_est.cols; ++jj) {
        float idepth = idepthmap(ii, jj);
        if (!std::isnan(idepth) && (idepth > 0)) {
          depth_est(ii, jj) = 1.0f / idepth;
        }
      }
    }

    if (publish_depthmap_) {
      publishDepthMap(depth_pub_, live_frame_id_, time, K_resized_, depth_est);
    }

    if (publish_cloud_) {
      float max_depth = (params_.do_idepth_triangle_filter) ? 1.0f / params_.min_triangle_idepth : std::numeric_limits<float>::max();
      publishPointCloud(cloud_pub_, live_frame_id_, time, K_resized_, depth_est, 0.1f, max_depth);
    }
  }

  if (publish_features_) {

    cv::Mat1f depth_raw(img_gray.rows, img_gray.cols, std::numeric_limits<float>::quiet_NaN());

    if (publish_features_) {
      std::vector<cv::Point2f> vertices;
      std::vector<float>       idepths_mu, idepths_var;
      sensor_->getRawIDepths(&vertices, &idepths_mu, &idepths_var);

      for (long unsigned int ii = 0; ii < vertices.size(); ++ii) {
        float id = idepths_mu[ii];
        // unused: float var = idepths_var[ii];
        if (!std::isnan(id) && (id > 0)) {
          int x = fu::fast_roundf(vertices[ii].x);
          int y = fu::fast_roundf(vertices[ii].y);

          FLAME_ASSERT(x >= 0);
          FLAME_ASSERT(x < depth_raw.cols);
          FLAME_ASSERT(y >= 0);
          FLAME_ASSERT(y < depth_raw.rows);

          depth_raw(y, x) = 1.0f / id;
        }
      }
    }

    publishDepthMap(features_pub_, live_frame_id_, time, K_resized_, depth_raw);
  }

  if (publish_stats_) {
    auto stats   = sensor_->stats().stats();
    auto timings = sensor_->stats().timings();
    publishFlameStats(stats_pub_, img_id, time, stats, timings);
  }

  stats_.set("latency", (clock_->now().seconds() - time) * 1000);

  if (!params_.debug_quiet) {
    RCLCPP_INFO(node_->get_logger(), "FlameRos/latency = %4.1fms\n", stats_.stats("latency"));
  }

  stats_.tock("publishing");

  if (!params_.debug_quiet) {
    RCLCPP_INFO(node_->get_logger(), "FlameRos/publishing = %4.1fms\n", stats_.timings("publishing"));
  }

  /*==================== Publish debug stuff ====================*/
  stats_.tick("debug_publishing");

  std_msgs::msg::Header hdr;
  hdr.stamp.sec     = time;
  hdr.stamp.nanosec = 0;
  hdr.frame_id      = live_frame_id_;

  if (params_.debug_draw_wireframe) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg = cv_bridge::CvImage(hdr, "bgr8", sensor_->getDebugImageWireframe()).toImageMsg();
    debug_wireframe_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_features) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg = cv_bridge::CvImage(hdr, "bgr8", sensor_->getDebugImageFeatures()).toImageMsg();
    debug_features_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_detections) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg = cv_bridge::CvImage(hdr, "bgr8", sensor_->getDebugImageDetections()).toImageMsg();
    debug_detections_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_matches) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg = cv_bridge::CvImage(hdr, "bgr8", sensor_->getDebugImageMatches()).toImageMsg();
    debug_matches_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_normals) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg = cv_bridge::CvImage(hdr, "bgr8", sensor_->getDebugImageNormals()).toImageMsg();
    debug_normals_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_idepthmap) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg = cv_bridge::CvImage(hdr, "bgr8", sensor_->getDebugImageInverseDepthMap()).toImageMsg();
    debug_idepthmap_pub_.publish(debug_img_msg);
  }

  stats_.tock("debug_publishing");

  if (!params_.debug_quiet) {
    RCLCPP_INFO(node_->get_logger(), "FlameRos/debug_publishing = %4.1fms\n", stats_.timings("debug_publishing"));
  }

  stats_.tock("process_frame");

  if (!params_.debug_quiet) {
    RCLCPP_INFO(node_->get_logger(), "FlameRos/process_frame = %4.1fms\n", stats_.timings("process_frame"));
  }

  return;
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackCamera() //{ */

void FlameRos::callbackCamera(const std::shared_ptr<const sensor_msgs::msg::Image> &rgb_msg, const std::shared_ptr<const sensor_msgs::msg::CameraInfo> &info) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting camera data");

  camera_frame_ = rgb_msg->header.frame_id;

  // Grab rgb data.
  cv::Mat3b rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;

  if (_undistort_fisheye_enabled_) {
    if (rgb.cols != _custom_fisheye_expected_width_ || rgb.rows != _custom_fisheye_expected_height_) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the received image resolution does not match the custom calibration file");
      return;
    }
  }

  assert(rgb.isContinuous());

  if (!inited_) {

    RCLCPP_INFO(node_->get_logger(), "initiating flame sensor object");

    live_frame_id_ = rgb_msg->header.frame_id;

    if (_camera_frame_ != "") {
      live_frame_id_ = _camera_frame_;
    }

    width_  = rgb.cols;
    height_ = rgb.rows;

    if (_undistort_fisheye_enabled_) {

      RCLCPP_INFO(node_->get_logger(), " ... using custom fisheye");

      for (int ii = 0; ii < 3; ++ii) {
        for (int jj = 0; jj < 3; ++jj) {
          K_(ii, jj) = _custom_fisheye_undistorted_K_.at<double>(ii, jj);
        }
      }

    } else {

      RCLCPP_INFO(node_->get_logger(), " ... using standard camera model");

      for (int ii = 0; ii < 3; ++ii) {
        for (int jj = 0; jj < 3; ++jj) {
          K_(ii, jj) = info->p[ii * 4 + jj];
        }
      }

      for (int ii = 0; ii < 5; ++ii) {
        D_(ii) = info->d[ii];
      }
    }

    K_resized_ = K_;

    RCLCPP_INFO_STREAM(node_->get_logger(), "K_resiszed = " << K_resized_ << std::endl);

    RCLCPP_INFO_STREAM(node_->get_logger(), "resize_factor_ = " << resize_factor_);

    K_resized_(0, 0) /= resize_factor_;
    K_resized_(0, 2) /= resize_factor_;
    K_resized_(1, 1) /= resize_factor_;
    K_resized_(1, 2) /= resize_factor_;

    RCLCPP_INFO_STREAM(node_->get_logger(), "K_resiszed = " << K_resized_ << std::endl);

    if (K_(0, 0) <= 1.0) {
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "Camera intrinsics matrix is probably invalid!");
      return;
    }

    Kinv_ = K_resized_.inverse();

    // Initialize depth sensor.
    if (!params_.debug_quiet) {
      RCLCPP_INFO(node_->get_logger(), "FlameRos: Constructing Flame...");
    }

    sensor_ = std::make_shared<flame::Flame>(width_ / resize_factor_, height_ / resize_factor_, K_resized_, Kinv_, params_);

    /*==================== Enter main loop ====================*/
    if (!params_.debug_quiet) {
      RCLCPP_INFO(node_->get_logger(), "FlameRos: Done. We are GO for launch!\n");
    }

    inited_ = true;
  }

  if (_undistort_enabled_) {

    if (!rgb.empty()) {

      if (_undistort_fisheye_enabled_) {

        // High-speed remapping
        cv::remap(rgb, rgb, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        RCLCPP_INFO_ONCE(node_->get_logger(), "rectifying using custom fisheye model");

      } else {

        cv::Mat1f Kcv, Dcv;
        cv::eigen2cv(K_, Kcv);
        cv::eigen2cv(D_, Dcv);
        cv::Mat3b rgb_undistorted;
        cv::undistort(rgb, rgb_undistorted, Kcv, Dcv);

        rgb = rgb_undistorted;

        RCLCPP_INFO_ONCE(node_->get_logger(), "rectifying using standard polynomial model");
      }

      publishUndistortedImage(rgb, rgb_msg->header);
    }
  }

  if (resize_factor_ != 1) {
    cv::Mat3b resized_rgb(static_cast<float>(rgb.rows) / resize_factor_, static_cast<float>(rgb.cols) / resize_factor_);
    cv::resize(rgb, resized_rgb, resized_rgb.size());
    rgb = resized_rgb;
  }

  // Get pose of camera.
  geometry_msgs::msg::TransformStamped tf;

  std::string cam_frame = rgb_msg->header.frame_id;

  if (_camera_frame_ != "") {
    cam_frame = _camera_frame_;
  }

  auto tf_opt = transformer_->getTransform(cam_frame, _world_frame_, rclcpp::Time(rgb_msg->header.stamp));

  if (!tf_opt) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "failed to get tf from %s to %s", cam_frame.c_str(), _world_frame_.c_str());
    return;
  }

  Sophus::SE3f pose;

  ros_sensor_streams::tfToSophusSE3<float>(tf_opt.value().transform, &pose);

  processFrame(frame_counter++, live_frame_id_, rclcpp::Time(rgb_msg->header.stamp).seconds(), pose, rgb);
}

//}

// | ----------------------- publishers ----------------------- |

/* publishUndistortedImage() //{ */

void FlameRos::publishUndistortedImage(const cv::Mat3b &rgb, const std_msgs::msg::Header &orig_header) {

  if (!publish_undistort_) {
    return;
  }

  sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg = cv_bridge::CvImage(orig_header, "bgr8", rgb).toImageMsg();

  sensor_msgs::msg::CameraInfo cam_info;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cam_info.k[i + j * 3] = K_resized_(i, j);
    }
  }

  cam_info.header = orig_header;

  cam_info.distortion_model = "plumb_bob";

  cam_info.height = rgb.rows;
  cam_info.width  = rgb.cols;

  cam_info.d.resize(5);
  cam_info.d[0] = 0;
  cam_info.d[1] = 0;
  cam_info.d[2] = 0;
  cam_info.d[3] = 0;
  cam_info.d[4] = 0;

  // rectification
  cam_info.r[0] = 1.0;
  cam_info.r[1] = 0.0;
  cam_info.r[2] = 0.0;
  cam_info.r[3] = 0.0;
  cam_info.r[4] = 1.0;
  cam_info.r[5] = 0.0;
  cam_info.r[6] = 0.0;
  cam_info.r[7] = 0.0;
  cam_info.r[8] = 1.0;

  cam_info.p[0]  = cam_info.k[0];
  cam_info.p[1]  = 0.0;
  cam_info.p[2]  = cam_info.k[2];
  cam_info.p[3]  = 0.0;
  cam_info.p[4]  = 0.0;
  cam_info.p[5]  = cam_info.k[4];
  cam_info.p[6]  = cam_info.k[5];
  cam_info.p[7]  = 0.0;
  cam_info.p[8]  = 0.0;
  cam_info.p[9]  = 0.0;
  cam_info.p[10] = 1.0;
  cam_info.p[11] = 0.0;

  debug_undistort_.publish(*debug_img_msg, cam_info);
}

//}

/* publishFlameNodeletStats() //{ */

void FlameRos::publishFlameNodeletStats(mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameNodeletStats> &pub, int img_id, double time,
                                        const std::unordered_map<std::string, double> &stats, const std::unordered_map<std::string, double> &timings) {

  flame_ros_msgs::msg::FlameNodeletStats::SharedPtr msg(new flame_ros_msgs::msg::FlameNodeletStats());

  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME);

  msg->img_id    = img_id;
  msg->timestamp = time;

  // Fill stat if it exists in the map.
  auto fillStati = [](const std::unordered_map<std::string, double> &stats, const std::string &name, int *out) {
    if (stats.count(name) > 0) {
      *out = stats.at(name);
    }
    return;
  };
  auto fillStatf = [](const std::unordered_map<std::string, double> &stats, const std::string &name, float *out) {
    if (stats.count(name) > 0) {
      *out = stats.at(name);
    }
    return;
  };

  fillStati(stats, "queue_size", &msg->queue_size);
  fillStatf(stats, "fps", &msg->fps);
  fillStatf(stats, "fps_max", &msg->fps_max);
  fillStatf(timings, "main", &msg->main_ms);
  fillStatf(timings, "waiting", &msg->waiting_ms);
  fillStatf(timings, "process_frame", &msg->process_frame_ms);
  fillStatf(timings, "publishing", &msg->publishing_ms);
  fillStatf(timings, "debug_publishing", &msg->debug_publishing_ms);
  fillStatf(stats, "latency", &msg->latency_ms);

  fillStatf(stats, "max_load_cpu", &msg->max_load_cpu);
  fillStatf(stats, "max_load_mem", &msg->max_load_mem);
  fillStatf(stats, "max_load_swap", &msg->max_load_swap);
  fillStatf(stats, "sys_load_cpu", &msg->sys_load_cpu);
  fillStatf(stats, "sys_load_mem", &msg->sys_load_mem);
  fillStatf(stats, "sys_load_swap", &msg->sys_load_swap);
  fillStatf(stats, "pid_load_cpu", &msg->pid_load_cpu);
  fillStatf(stats, "pid_load_mem", &msg->pid_load_mem);
  fillStatf(stats, "pid_load_swap ", &msg->pid_load_swap);
  fillStati(stats, "pid", &msg->pid);

  pub.publish(*msg);

  return;
}

//}

/* publishFlameStats() //{ */

void FlameRos::publishFlameStats(mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameStats> &pub, int img_id, double time,
                                 const std::unordered_map<std::string, double> &stats, const std::unordered_map<std::string, double> &timings) {

  flame_ros_msgs::msg::FlameStats::SharedPtr msg(new flame_ros_msgs::msg::FlameStats());

  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME);

  msg->img_id    = img_id;
  msg->timestamp = time;

  // Fill stat if it exists in the map.
  auto fillStati = [](const std::unordered_map<std::string, double> &stats, const std::string &name, int *out) {
    if (stats.count(name) > 0) {
      *out = stats.at(name);
    }
    return;
  };

  auto fillStatf = [](const std::unordered_map<std::string, double> &stats, const std::string &name, float *out) {
    if (stats.count(name) > 0) {
      *out = stats.at(name);
    }
    return;
  };

  fillStati(stats, "num_feats", &msg->num_feats);
  fillStati(stats, "num_vtx", &msg->num_vtx);
  fillStati(stats, "num_tris", &msg->num_tris);
  fillStati(stats, "num_edges", &msg->num_edges);

  fillStatf(stats, "coverage", &msg->coverage);

  fillStati(stats, "num_idepth_updates", &msg->num_idepth_updates);
  fillStati(stats, "num_fail_max_var", &msg->num_fail_max_var);
  fillStati(stats, "num_fail_max_dropouts", &msg->num_fail_max_dropouts);
  fillStati(stats, "num_fail_ref_patch_grad", &msg->num_fail_ref_patch_grad);
  fillStati(stats, "num_fail_ambiguous_match", &msg->num_fail_ambiguous_match);
  fillStati(stats, "num_fail_max_cost", &msg->num_fail_max_cost);

  fillStatf(stats, "nltgv2_total_smoothness_cost", &msg->nltgv2_total_smoothness_cost);
  fillStatf(stats, "nltgv2_avg_smoothness_cost", &msg->nltgv2_avg_smoothness_cost);
  fillStatf(stats, "nltgv2_total_data_cost", &msg->nltgv2_total_data_cost);
  fillStatf(stats, "nltgv2_avg_data_cost", &msg->nltgv2_avg_data_cost);

  fillStatf(stats, "total_photo_error", &msg->total_photo_error);
  fillStatf(stats, "avg_photo_error", &msg->avg_photo_error);

  fillStatf(stats, "fps", &msg->fps);
  fillStatf(stats, "fps_max", &msg->fps_max);
  fillStatf(timings, "update", &msg->update_ms);
  fillStatf(timings, "update_locking", &msg->update_locking_ms);
  fillStatf(timings, "frame_creation", &msg->frame_creation_ms);
  fillStatf(timings, "interpolate", &msg->interpolate_ms);
  fillStatf(timings, "keyframe", &msg->keyframe_ms);
  fillStatf(timings, "detection", &msg->detection_ms);
  fillStatf(timings, "detection_loop", &msg->detection_loop_ms);
  fillStatf(timings, "update_idepths", &msg->update_idepths_ms);
  fillStatf(timings, "project_features", &msg->project_features_ms);
  fillStatf(timings, "project_graph", &msg->project_graph_ms);
  fillStatf(timings, "sync_graph", &msg->sync_graph_ms);
  fillStatf(timings, "triangulate", &msg->triangulate_ms);
  fillStatf(timings, "median_filter", &msg->median_filter_ms);
  fillStatf(timings, "lowpass_filter", &msg->lowpass_filter_ms);

  pub.publish(*msg);

  return;
}

//}

/* publishDepthMesh() //{ */

void FlameRos::publishDepthMesh(mrs_lib::PublisherHandler<pcl_msgs::msg::PolygonMesh> &mesh_pub, const std::string &frame_id, double time,
                                const Eigen::Matrix3f &Kinv, const std::vector<cv::Point2f> &vertices, const std::vector<float> &idepths,
                                const std::vector<Eigen::Vector3f> &normals, const std::vector<flame::Triangle> &triangles,
                                const std::vector<bool> &tri_validity, const cv::Mat3b &rgb) {

  pcl_msgs::msg::PolygonMesh::SharedPtr msg(new pcl_msgs::msg::PolygonMesh());

  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME);

  msg->header.frame_id = frame_id;

  // Create point cloud to hold vertices.
  pcl::PointCloud<flame_ros::PointNormalUV> cloud;

  cloud.width  = vertices.size();
  cloud.height = 1;
  cloud.points.resize(vertices.size());
  cloud.is_dense = false;

  for (long unsigned int ii = 0; ii < vertices.size(); ++ii) {

    float id = idepths[ii];

    if (!std::isnan(id) && (id > 0.0f)) {

      Eigen::Vector3f uhom(vertices[ii].x, vertices[ii].y, 1.0f);
      uhom /= id;
      Eigen::Vector3f p(Kinv * uhom);
      cloud.points[ii].x = p(0);
      cloud.points[ii].y = p(1);
      cloud.points[ii].z = p(2);

      cloud.points[ii].normal_x = normals[ii](0);
      cloud.points[ii].normal_y = normals[ii](1);
      cloud.points[ii].normal_z = normals[ii](2);

      // OpenGL textures range from 0 to 1.
      cloud.points[ii].u = vertices[ii].x / (rgb.cols - 1);
      cloud.points[ii].v = vertices[ii].y / (rgb.rows - 1);

    } else {

      // Add invalid value to skip this point. Note that the initial value
      // is (0, 0, 0), so you must manually invalidate the point.
      cloud.points[ii].x = std::numeric_limits<float>::quiet_NaN();
      cloud.points[ii].y = std::numeric_limits<float>::quiet_NaN();
      cloud.points[ii].z = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
  }

  pcl::toROSMsg(cloud, msg->cloud);

  // NOTE: Header fields need to be filled in after pcl::toROSMsg() call.
  msg->cloud.header = std_msgs::msg::Header();

  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME);

  msg->cloud.header.frame_id = frame_id;

  // Fill in faces.
  msg->polygons.reserve(triangles.size());

  for (long unsigned int ii = 0; ii < triangles.size(); ++ii) {

    if (tri_validity[ii]) {

      pcl_msgs::msg::Vertices vtx_ii;
      vtx_ii.vertices.resize(3);
      vtx_ii.vertices[0] = triangles[ii][2];
      vtx_ii.vertices[1] = triangles[ii][1];
      vtx_ii.vertices[2] = triangles[ii][0];

      msg->polygons.push_back(vtx_ii);
    }
  }

  if (msg->polygons.size() > 0) {
    mesh_pub.publish(*msg);
  }
}

//}

/* publishDepthMap() //{ */

void FlameRos::publishDepthMap(const image_transport::CameraPublisher &pub, const std::string &frame_id, double time, const Eigen::Matrix3f &K,
                               const cv::Mat1f &depth_est) {

  // Publish depthmap.
  std_msgs::msg::Header header;

  header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME);

  header.frame_id = frame_id;

  sensor_msgs::msg::CameraInfo::SharedPtr cinfo(new sensor_msgs::msg::CameraInfo);
  cinfo->header           = header;
  cinfo->height           = depth_est.rows;
  cinfo->width            = depth_est.cols;
  cinfo->distortion_model = "plumb_bob";
  cinfo->d                = {0.0, 0.0, 0.0, 0.0, 0.0};

  for (int ii = 0; ii < 3; ++ii) {
    for (int jj = 0; jj < 3; ++jj) {
      cinfo->k[ii * 3 + jj] = K(ii, jj);
      cinfo->p[ii * 4 + jj] = K(ii, jj);
      cinfo->r[ii * 3 + jj] = 0.0;
    }
  }

  cinfo->p[3]  = 0.0;
  cinfo->p[7]  = 0.0;
  cinfo->p[11] = 0.0;
  cinfo->r[0]  = 1.0;
  cinfo->r[4]  = 1.0;
  cinfo->r[8]  = 1.0;

  cv_bridge::CvImage depth_cvi(header, "32FC1", depth_est);

  pub.publish(depth_cvi.toImageMsg(), cinfo);
}

//}

/* publishPointCloud() //{ */

void FlameRos::publishPointCloud(mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> &pub, const std::string &frame_id, double time,
                                 const Eigen::Matrix3f &K, const cv::Mat1f &depth_est, float min_depth, float max_depth) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  int height = depth_est.rows;
  int width  = depth_est.cols;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  cloud->width    = width;
  cloud->height   = height;
  cloud->is_dense = false;
  cloud->points.resize(width * height);

  Eigen::Matrix3f Kinv(K.inverse());

  for (int ii = 0; ii < height; ++ii) {
    for (int jj = 0; jj < width; ++jj) {

      int idx = ii * width + jj;

      float depth = depth_est(ii, jj);

      if (std::isnan(depth) || (depth < min_depth) || (depth > max_depth)) {
        // Add invalid value to skip this point. Note that the initial value
        // is (0, 0, 0), so you must manually invalidate the point.
        cloud->points[idx].x = std::numeric_limits<float>::quiet_NaN();
        cloud->points[idx].y = std::numeric_limits<float>::quiet_NaN();
        cloud->points[idx].z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      Eigen::Vector3f xyz(jj * depth, ii * depth, depth);
      xyz = Kinv * xyz;

      cloud->points[idx].x = xyz(0);
      cloud->points[idx].y = xyz(1);
      cloud->points[idx].z = xyz(2);
    }
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

  voxel_filter.setInputCloud(cloud);

  voxel_filter.setLeafSize(drs_params.cloud_decimation, drs_params.cloud_decimation, drs_params.cloud_decimation);

  voxel_filter.filter(*cloud);

  sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2());
  pcl::toROSMsg(*cloud, *msg);

  msg->header = std_msgs::msg::Header();

  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME);

  msg->header.frame_id = frame_id;

  pub.publish(*msg);

  return;
}

//}

// | --------------------- other routines --------------------- |

/* getDepthConfusionMatrix() //{ */

void getDepthConfusionMatrix(const cv::Mat1f &idepths, const cv::Mat1f &depth, cv::Mat1f *idepth_error, float *total_error, int *true_pos, int *true_neg,
                             int *false_pos, int *false_neg) {

  // Compute confusion matrix with detection being strictly positive idepth.
  *true_pos  = 0;
  *true_neg  = 0;
  *false_pos = 0;
  *false_neg = 0;

  *total_error  = 0.0f;
  *idepth_error = cv::Mat1f(depth.rows, depth.cols, std::numeric_limits<float>::quiet_NaN());

  for (int ii = 0; ii < depth.rows; ++ii) {
    for (int jj = 0; jj < depth.cols; ++jj) {

      if (depth(ii, jj) > 0) {

        if (!std::isnan(idepths(ii, jj))) {

          float idepth_est  = idepths(ii, jj);
          float idepth_true = 1.0f / depth(ii, jj);

          float error             = fu::fast_abs(idepth_est - idepth_true);
          (*idepth_error)(ii, jj) = error;
          *total_error += error;

          (*true_pos)++;
        } else {
          (*false_neg)++;
        }

      } else if (!std::isnan(idepths(ii, jj))) {

        float idepth_est        = idepths(ii, jj);
        float error             = fu::fast_abs(idepth_est);
        (*idepth_error)(ii, jj) = error;
        *total_error += error;

        (*false_pos)++;

      } else {
        (*true_neg)++;
      }
    }
  }
}

//}

} // namespace flame_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(flame_ros::FlameRos)
