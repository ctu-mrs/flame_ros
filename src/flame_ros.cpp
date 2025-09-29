#include <flame_ros/flame_ros.hpp>

#define NODE_NAME "flame"

namespace flame_ros {

FlameRos::FlameRos(const rclcpp::NodeOptions & options) :
  rclcpp::Node(NODE_NAME, options),
  is_initialized_(false),
  tf_listener_(nullptr),
  tf_buffer_(get_clock()),
  resize_factor_(1),
  use_external_cal_(false),
  odom_path(nullptr),
  pose_frame_id(0),
  inited_(false),
  undistort_(false),
  live_frame_id_(),
  width_(0),
  height_(0),
  K_(),
  D_(5),
  cam_sub_()
{
  timer_initialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&FlameRos::timerInitialization, this));
}

void FlameRos::timerInitialization() {
  // std::signal(SIGSEGV, crash_handler);
  // std::signal(SIGILL, crash_handler);
  // std::signal(SIGABRT, crash_handler);
  // std::signal(SIGFPE, crash_handler);

  image_transport_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());

  mrs_lib::ParamLoader param_loader(shared_from_this(), NODE_NAME);

  load_ = std::move(fu::LoadTracker(getpid()));

  num_imgs_ = 0;

  /*==================== Input Params ====================*/
  param_loader.loadParam("input.camera_world_frame_id", camera_world_frame_id_);
  param_loader.loadParam("input.subsample_factor", subsample_factor_);
  param_loader.loadParam("input.poseframe_subsample_factor", poseframe_subsample_factor_);
  param_loader.loadParam("input.use_poseframe_updates", use_poseframe_updates_);
  param_loader.loadParam("input.poseframe_child_frame_id", poseframe_child_frame_id_);
  param_loader.loadParam("input.resize_factor", resize_factor_);

  /*==================== Output Params ====================*/
  param_loader.loadParam("output.quiet", params_.debug_quiet);
  param_loader.loadParam("output.mesh", publish_mesh_);
  param_loader.loadParam("output.idepthmap", publish_idepthmap_);
  param_loader.loadParam("output.depthmap", publish_depthmap_);
  param_loader.loadParam("output.cloud", publish_cloud_);
  param_loader.loadParam("output.features", publish_features_);
  param_loader.loadParam("output.stats", publish_stats_);
  param_loader.loadParam("output.load_integration_factor", load_integration_factor_);
  param_loader.loadParam("output.scene_color_scale", params_.scene_color_scale);
  param_loader.loadParam("output.filter_oblique_triangles", params_.do_oblique_triangle_filter);

  double oblique_normal_thresh;
  param_loader.loadParam("output.oblique_normal_thresh", oblique_normal_thresh);
  params_.oblique_normal_thresh = oblique_normal_thresh;

  param_loader.loadParam("output.oblique_idepth_diff_factor", params_.oblique_idepth_diff_factor);
  param_loader.loadParam("output.oblique_idepth_diff_abs", params_.oblique_idepth_diff_abs);
  param_loader.loadParam("output.filter_long_edges", params_.do_edge_length_filter);

  double edge_length_thresh;
  param_loader.loadParam("output.edge_length_thresh", edge_length_thresh);
  params_.edge_length_thresh = edge_length_thresh;

  param_loader.loadParam("output.filter_triangles_by_idepth", params_.do_idepth_triangle_filter);

  double min_triangle_idepth;
  param_loader.loadParam("output.min_triangle_idepth", min_triangle_idepth);
  params_.min_triangle_idepth = min_triangle_idepth;

  param_loader.loadParam("output.max_angular_rate", max_angular_rate_);

  /*==================== Debug Params ====================*/
  param_loader.loadParam("debug.wireframe", params_.debug_draw_wireframe);
  param_loader.loadParam("debug.features", params_.debug_draw_features);
  param_loader.loadParam("debug.detections", params_.debug_draw_detections);
  param_loader.loadParam("debug.matches", params_.debug_draw_matches);
  param_loader.loadParam("debug.normals", params_.debug_draw_normals);
  param_loader.loadParam("debug.idepthmap", params_.debug_draw_idepthmap);
  param_loader.loadParam("debug.text_overlay", params_.debug_draw_text_overlay);
  param_loader.loadParam("debug.flip_images", params_.debug_flip_images);

  /*==================== Threading Params ====================*/
  param_loader.loadParam("threading.openmp.num_threads", params_.omp_num_threads);
  param_loader.loadParam("threading.openmp.chunk_size", params_.omp_chunk_size);

  /*==================== Feature Params ====================*/
  param_loader.loadParam("features.do_letterbox", params_.do_letterbox);
  param_loader.loadParam("features.detection.min_grad_mag", params_.min_grad_mag);
  params_.fparams.min_grad_mag = params_.min_grad_mag;

  double min_error;
  param_loader.loadParam("features.detection.min_error", min_error);
  params_.min_error = min_error;

  param_loader.loadParam("features.detection.win_size", params_.detection_win_size);

  int win_size;
  param_loader.loadParam("features.tracking.win_size", win_size);
  params_.zparams.win_size = win_size;
  params_.fparams.win_size = win_size;

  param_loader.loadParam("features.tracking.max_dropouts", params_.max_dropouts);

  double epipolar_line_var;
  param_loader.loadParam("features.tracking.epipolar_line_var", epipolar_line_var);
  params_.zparams.epipolar_line_var = epipolar_line_var;

  /*==================== Regularization Params ====================*/
  param_loader.loadParam("regularization.do_nltgv2", params_.do_nltgv2);
  param_loader.loadParam("regularization.nltgv2.adaptive_data_weights", params_.adaptive_data_weights);
  param_loader.loadParam("regularization.nltgv2.rescale_data", params_.rescale_data);
  param_loader.loadParam("regularization.nltgv2.init_with_prediction", params_.init_with_prediction);
  param_loader.loadParam("regularization.nltgv2.idepth_var_max", params_.idepth_var_max_graph);
  param_loader.loadParam("regularization.nltgv2.data_factor", params_.rparams.data_factor);
  param_loader.loadParam("regularization.nltgv2.step_x", params_.rparams.step_x);
  param_loader.loadParam("regularization.nltgv2.step_q", params_.rparams.step_q);
  param_loader.loadParam("regularization.nltgv2.theta", params_.rparams.theta);
  param_loader.loadParam("regularization.nltgv2.min_height", params_.min_height);
  param_loader.loadParam("regularization.nltgv2.max_height", params_.max_height);
  param_loader.loadParam("regularization.nltgv2.check_sticky_obstacles", params_.check_sticky_obstacles);

#ifdef FLAME_WITH_FLA
  bool use_camera_info = false;
  param_loader.loadParam("input.use_camera_info", &use_camera_info);

  if (use_camera_info) {
    // Make sure we don't attempt to resize the image.
    FLAME_ASSERT(resize_factor_ == 1);

    // Setup input stream.
    input_ = std::make_shared<ros_sensor_streams::
        TrackedImageStream>(camera_world_frame_id_, nh);
  } else {
    // FLA does not follow ROS conventions for camera/image streams/calibration,
    // so we need to do some extra work to pass data through. First, the
    // camera_info messages containing the camera intrinsics/distortion are not
    // filled in. We need to grab the calibration from samros's params. Since
    // samwise estimates the paramters on the fly, we should really use the
    // refined versions, but this should work for now. Finally, we need to
    // undistort the images.
    int width, height;
    param_loader.loadParam("samros.camera.image_width", &width);
    param_loader.loadParam("samros.camera.image_height", &height);

    if (((width != 640) && (width != 1280)) ||
        ((height != 512) && (height != 1024))) {
      RCLCPP_ERROR(get_logger(), "FlameRos: Unexpected image size = (%i, % i)\n",
                width, height);
    }

    double fx, fy, cx, cy;
    param_loader.loadParam("samros.camera.intrinsics.fu", &fx);
    param_loader.loadParam("samros.camera.intrinsics.fv", &fy);
    param_loader.loadParam("samros.camera.intrinsics.pu", &cx);
    param_loader.loadParam("samros.camera.intrinsics.pv", &cy);

    double k1, k2, p1, p2, k3;
    param_loader.loadParam("samros.camera.distortion.k1", &k1);
    param_loader.loadParam("samros.camera.distortion.k2", &k2);
    param_loader.loadParam("samros.camera.distortion.p1", &p1);
    param_loader.loadParam("samros.camera.distortion.p2", &p2);
    param_loader.loadParam("samros.camera.distortion.k3", &k3);

    Eigen::VectorXf D(5);
    D << k1, k2, p1, p2, k3;

    Eigen::Matrix3f K(Eigen::Matrix3f::Zero());
    K(0, 0) = fx;
    K(0, 2) = cx;
    K(1, 1) = fy;
    K(1, 2) = cy;
    K(2, 2) = 1.0f;

    // Adjust calibration based on resize_factor.
    K /= resize_factor_;
    K(2, 2) = 1.0f;

    // Setup input stream.
    bool undistort = true;
    input_ = std::make_shared<ros_sensor_streams::
                              TrackedImageStream>(camera_world_frame_id_, nh, K,
                                                  D, undistort, resize_factor_);
  }

  // Setup health and status.
  int tmp; // getParam can't handle uint8_t.
  param_loader.loadParam("fla.node_id", &tmp);
  node_id_ = tmp;

  param_loader.loadParam("fla.heart_beat_dt", &heart_beat_dt_);
  param_loader.loadParam("fla.alarm_timeout", &alarm_timeout_);
  param_loader.loadParam("fla.fail_timeout", &fail_timeout_);

  heart_beat_ = nh.createTimer(rclcpp::Duration(heart_beat_dt_, 0),
                                &FlameRos::heartBeat, this);
  heart_beat_pub_ = nh.advertise<fla_msgs::ProcessStatus>("/globalstatus", 1);
#else
  // Image resizing not supported for non-FLA.
  FLAME_ASSERT(resize_factor_ == 1);

  // Setup input stream.
  //input_ = std::make_shared<ros_sensor_streams::
  //                          TrackedImageStream>(camera_world_frame_id_, shared_from_this());
    // Subscribe to topics.
  //image_transport::ImageTransport it_(shared_from_this());
  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  image_transport_.reset(new image_transport::ImageTransport(shared_from_this()));

  cam_sub_ = image_transport_->subscribeCamera(std::string("~/image_in"), 10,
                                                [this](const sensor_msgs::msg::Image::ConstSharedPtr& img,
                                                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
                                                 this->callback(img, info);
                                             });

  // Set up tf.
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
#endif

  pfs_inited_ = true; // Default values.
  first_pf_id_ = 0;
  RCLCPP_INFO(get_logger(), "poseframe updated enabled: %s", use_poseframe_updates_ ? "true" : "false");
  if (use_poseframe_updates_) {
      // Wait until first pf message received with >= 2 pfs so that we can
      // estimate the poseframe subsample factor and offset.
      pfs_inited_ = false;

      // Subscribe to poseframe topic.
      //poseframe_sub_ = nh.subscribe("poseframes", 1,
      //                              &FlameRos::poseframeCallback, this);
      // mrs_lib::SubscriberHandlerOptions shopts(shared_from_this());
      // shopts.node_name = NODE_NAME;
      // shopts.topic_name = "pathimu";
      //shopts.no_message_timeout = no_message_timeout;
      // poseframe_sub_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Path>(
      //     shopts,
      //     std::bind(&FlameRos::poseframeCallback, this, std::placeholders::_1)
      // );

      poseframe_sub_ = create_subscription<nav_msgs::msg::Path>(
        "pathimu", 10, std::bind(&FlameRos::poseframeCallback, this, std::placeholders::_1));
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
         "odom", 10, std::bind(&FlameRos::odomCallback, this, std::placeholders::_1));
  }

  // Set up publishers. For some reason this appears to take a while.
  if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameRos: Setting up publishers...\n");

  //it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

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
    mesh_pub_ = mrs_lib::PublisherHandler<pcl_msgs::msg::PolygonMesh>(shared_from_this(), "~/mesh_out");
  }
  if (publish_cloud_) {
    //cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 5);
    cloud_pub_ = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(shared_from_this(), "~/cloud_out");
  }
  if (publish_stats_) {
    //stats_pub_ = nh.advertise<FlameStats>("stats", 5);
    stats_pub_ = mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameStats>(shared_from_this(), "~/stats_out");
    //nodelet_stats_pub_ = nh.advertise<FlameNodeletStats>("nodelet_stats", 5);
    nodelet_stats_pub_ = mrs_lib::PublisherHandler<flame_ros_msgs::msg::FlameNodeletStats>(shared_from_this(), "~/nodelet_stats_out");
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

  is_initialized_ = true;
  timer_initialization_->cancel();

  // Kick off main thread.
  //thread_ = std::thread(&FlameRos::main, this);

  RCLCPP_INFO(get_logger(), "flame_ros constructed");
}

void FlameRos::callback(const std::shared_ptr<const sensor_msgs::msg::Image>& rgb_msg,
                                  const std::shared_ptr<const sensor_msgs::msg::CameraInfo>& info) {
  RCLCPP_DEBUG(get_logger(), "Received image data!");

  // Grab rgb data.
  cv::Mat3b rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;

  assert(rgb.isContinuous());

  if (resize_factor_ != 1) {
    cv::Mat3b resized_rgb(static_cast<float>(rgb.rows)/resize_factor_,
                          static_cast<float>(rgb.cols)/resize_factor_);
    cv::resize(rgb, resized_rgb, resized_rgb.size());
    rgb = resized_rgb;
  }

  if (!inited_) {
    live_frame_id_ = rgb_msg->header.frame_id;

    // Set calibration.
    width_ = rgb.cols;
    height_ = rgb.rows;

    if (!use_external_cal_) {
      for (int ii = 0; ii < 3; ++ii) {
        for (int jj = 0; jj < 3; ++jj) {
          K_(ii, jj) = info->p[ii*4 + jj];
        }
      }

      if (K_(0, 0) <= 0) {
        RCLCPP_ERROR(get_logger(), "Camera intrinsics matrix is probably invalid!\n");
        RCLCPP_ERROR_STREAM(get_logger(), "K = " << std::endl << K_);
        return;
      }

      for (int ii = 0; ii < 5; ++ii) {
        D_(ii) = info->d[ii];
      }
    }

    inited_ = true;

    RCLCPP_DEBUG(get_logger(), "Set camera calibration!");

    Kinv_ = K_.inverse();

    // Initialize depth sensor.
    if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameRos: Constructing Flame...\n");
    sensor_ = std::make_shared<flame::Flame>(width_,
                                             height_,
                                             K_,
                                             Kinv_,
                                             params_);

    /*==================== Enter main loop ====================*/
    if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameRos: Done. We are GO for launch!\n");
  }

  if (undistort_) {
    cv::Mat1f Kcv, Dcv;
    cv::eigen2cv(K_, Kcv);
    cv::eigen2cv(D_, Dcv);
    cv::Mat3b rgb_undistorted;
    cv::undistort(rgb, rgb_undistorted, Kcv, Dcv);
    rgb = rgb_undistorted;
  }

  // Get pose of camera.
  geometry_msgs::msg::TransformStamped tf;
  try {
    // Need to remove leading "/" if it exists.
    std::string rgb_frame_id = rgb_msg->header.frame_id;
    if (rgb_frame_id[0] == '/') {
      rgb_frame_id = rgb_frame_id.substr(1, rgb_frame_id.size()-1);
    }

    tf = tf_buffer_.lookupTransform(camera_world_frame_id_, rgb_frame_id,
                                    rclcpp::Time(rgb_msg->header.stamp),
                                    rclcpp::Duration(1.0/10, 0));
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return;
  }

  Sophus::SE3f pose;

  ros_sensor_streams::tfToSophusSE3<float>(tf.transform, &pose);

  processFrame(frame_counter++, live_frame_id_, rclcpp::Time(rgb_msg->header.stamp).seconds(), Sophus::SE3f(pose.unit_quaternion(), pose.translation()),
                     rgb);

  return;
}

  /**
   * @brief Callback for receiving poseframe poses.
   */
void FlameRos::poseframeCallback(const nav_msgs::msg::Path::ConstSharedPtr msg) {
    CHECK_INIT

    if(camera_frame_id_.empty()){
      RCLCPP_WARN(get_logger(), "cam frame id was not obtained yet");
      return;
    }

    pose_frame_id++;
    if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameRos: Got a poseframe message!\n");
    RCLCPP_INFO(get_logger(), "FlameRos: Got a poseframe message!\n");

    // Get transform to camera_world.
    geometry_msgs::msg::TransformStamped tf;
    try {
      // Need to remove leading "/" if it exists.
      std::string frame_id = msg->header.frame_id;
      if (frame_id[0] == '/') {
        frame_id = frame_id.substr(1, frame_id.size()-1);
      }
      tf = tf_buffer_.lookupTransform(camera_world_frame_id_, frame_id,
                                      rclcpp::Time(msg->header.stamp),
                                      rclcpp::Duration(1.0/15, 0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      return;
    }

    Sophus::SE3f to_camera_world;
    ros_sensor_streams::tfToSophusSE3<float>(tf.transform, &to_camera_world);

    // Get transform of camera wrt body.
    try {
      // Need to remove leading "/" if it exists.
      std::string frame_id = msg->header.frame_id;
      if (frame_id[0] == '/') {
        frame_id = frame_id.substr(1, frame_id.size()-1);
      }
      tf = tf_buffer_.lookupTransform(poseframe_child_frame_id_, camera_frame_id_,
                                      rclcpp::Time(msg->header.stamp),
                                      rclcpp::Duration(1.0/15, 0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      return;
    }

    Sophus::SE3f to_camera;
    ros_sensor_streams::tfToSophusSE3<float>(tf.transform, &to_camera);

    // Extract ids and poses.
    std::vector<uint32_t> pf_ids(msg->poses.size());
    std::vector<Sophus::SE3f> pf_poses(msg->poses.size());
    for (long unsigned int ii = 0; ii < msg->poses.size(); ++ii) {
      pf_ids[ii] = pose_frame_id;

      Sophus::SE3f pose;
      ros_sensor_streams::poseToSophusSE3<float>(msg->poses[ii].pose, &pose);

      // Convert to camera world.
      pf_poses[ii] = to_camera_world * pose * to_camera;
    }

    if (!pfs_inited_ && (pf_ids.size() >= 2)) {
      // Initialize pf offset and subsample factor.
      first_pf_id_ = pf_ids[0];
      poseframe_subsample_factor_ = pf_ids[1] - pf_ids[0];
      pfs_inited_ = true;
    } else if ((sensor_ != nullptr) && pfs_inited_) {
      sensor_->updatePoseFramePoses(pf_ids, pf_poses);
      sensor_->prunePoseFrames(pf_ids);
      RCLCPP_INFO(get_logger(), "pruning");
    }

    return;
}

void FlameRos::append_odom_to_path(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
  if (!odom_path) {
      odom_path = std::make_shared<nav_msgs::msg::Path>();
  }
  // Update path header with current timestamp and frame
  odom_path->header.stamp.sec = odom_msg->header.stamp.sec;
  odom_path->header.stamp.nanosec = odom_msg->header.stamp.nanosec;
  odom_path->header.frame_id = odom_msg->header.frame_id;

  // Extract pose from odometry
  const auto& current_pose = odom_msg->pose.pose;

  // Check if we should add this pose based on distance threshold
  //if (should_add_pose(current_pose))
  //{
      // Create PoseStamped message
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = odom_msg->header;
      pose_stamped.pose = current_pose;

      // Add to path
      odom_path->poses.push_back(pose_stamped);

      // Update last pose
      //last_pose_ = current_pose;
      //has_last_pose_ = true;

      // Limit path length to prevent memory issues
      //if (static_cast<int>(odom_path->poses.size()) > max_path_length_)
      //{
          odom_path->poses.erase(odom_path->poses.begin());  // Remove oldest pose
      //}

      RCLCPP_DEBUG(this->get_logger(), "Added pose to path. Total poses: %zu", 
                  odom_path->poses.size());
  //}
}

void FlameRos::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  append_odom_to_path(odom_msg);
  poseframeCallback(odom_path);
}

void FlameRos::processFrame(const uint32_t img_id, const std::string& cam_frame_id, const double time,
                  const Sophus::SE3f& pose, const cv::Mat3b& rgb) {
  stats_.tick("process_frame");

  /*==================== Process image ====================*/
  // Convert to grayscale.
  cv::Mat1b img_gray;
  cv::cvtColor(rgb, img_gray, cv::COLOR_RGB2GRAY);

  bool is_poseframe = ((static_cast<int>(img_id) -  first_pf_id_) %
                        poseframe_subsample_factor_) == 0;
  std::string msg;
  bool update_success = sensor_->update(time, img_id, pose, img_gray,
                                        is_poseframe, msg=msg);


  if (!update_success) {
    stats_.tock("process_frame");
    if(!params_.debug_quiet) RCLCPP_WARN(get_logger(), "FlameRos: Unsuccessful update. Reason: %s\n", msg.c_str());
    return;
  }

  // | ------------------ custom pruning starts ----------------- |

  poses_ids_.insert(poses_ids_.begin(), img_id);

  if (poses_ids_.size() > 100) {

    poses_ids_.pop_back();

    sensor_->prunePoseFrames(poses_ids_);
  }

  // | ------------------- custom pruning ends ------------------ |

  if (max_angular_rate_ > 0.0f) {
    // Check angle difference between last and current pose. If we're rotating,
    // we shouldn't publish output since it's probably too noisy.
    Eigen::Quaternionf q_delta = pose.unit_quaternion() *
        prev_pose_.unit_quaternion().inverse();
    float angle_delta = fu::fast_abs(Eigen::AngleAxisf(q_delta).angle());
    float angle_rate = angle_delta / (time - prev_time_);

    prev_time_ = time;
    prev_pose_ = pose;

    if (angle_rate * 180.0f / M_PI > max_angular_rate_) {
      // Angular rate is too high.
      if(!params_.debug_quiet) RCLCPP_ERROR(get_logger(),
                      "Angle Delta = %.3f, rate = %f.3\n", angle_delta * 180.0f / M_PI,
                      angle_rate * 180.0f / M_PI);
      return;
    }
  }

  /*==================== Publish output ====================*/
  stats_.tick("publishing");

  if (publish_mesh_) {
    // Get current mesh.
    std::vector<cv::Point2f> vtx;
    std::vector<float> idepths;
    std::vector<Eigen::Vector3f> normals;
    std::vector<flame::Triangle> triangles;
    std::vector<flame::Edge> edges;
    std::vector<bool> tri_validity;
    sensor_->getInverseDepthMesh(&vtx, &idepths, &normals, &triangles,
                                  &tri_validity, &edges);

    publishDepthMesh(mesh_pub_, cam_frame_id, time, Kinv_, vtx,
                      idepths, normals, triangles, tri_validity, rgb);
  }

  if (publish_idepthmap_ || publish_depthmap_ || publish_cloud_) {
    cv::Mat1f idepthmap;
    sensor_->getFilteredInverseDepthMap(&idepthmap);

    if (publish_idepthmap_) {
      // Publish full idepthmap.
      publishDepthMap(idepth_pub_, cam_frame_id, time, K_,
                      sensor_->getInverseDepthMap());
    }

    // Convert to depths.
    cv::Mat1f depth_est(idepthmap.rows, idepthmap.cols,
                        std::numeric_limits<float>::quiet_NaN());
#pragma omp parallel for collapse(2) num_threads(params_.omp_num_threads) schedule(dynamic, params_.omp_chunk_size) // NOLINT
    for (int ii = 0; ii < depth_est.rows; ++ii) {
      for (int jj = 0; jj < depth_est.cols; ++jj) {
        float idepth =  idepthmap(ii, jj);
        if (!std::isnan(idepth) && (idepth > 0)) {
          depth_est(ii, jj) = 1.0f/ idepth;
        }
      }
    }

    if (publish_depthmap_) {
      publishDepthMap(depth_pub_, live_frame_id_, time, K_,
                      depth_est);
    }

    if (publish_cloud_) {
      float max_depth = (params_.do_idepth_triangle_filter) ?
          1.0f / params_.min_triangle_idepth : std::numeric_limits<float>::max();
      publishPointCloud(cloud_pub_, live_frame_id_, time, K_,
                        depth_est, 0.1f, max_depth);
    }
  }

  if (publish_features_) {
    cv::Mat1f depth_raw(img_gray.rows, img_gray.cols,
                        std::numeric_limits<float>::quiet_NaN());
    if (publish_features_) {
      std::vector<cv::Point2f> vertices;
      std::vector<float> idepths_mu, idepths_var;
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

    publishDepthMap(features_pub_, live_frame_id_, time, K_,
                    depth_raw);
  }

  if (publish_stats_) {
    auto stats = sensor_->stats().stats();
    auto timings = sensor_->stats().timings();
    publishFlameStats(stats_pub_, img_id, time, stats, timings);
  }

  stats_.set("latency", (get_clock()->now().seconds() - time) * 1000);
  if(!params_.debug_quiet) RCLCPP_INFO(get_logger(),
                    "FlameRos/latency = %4.1fms\n",
                    stats_.stats("latency"));

  stats_.tock("publishing");
  if(!params_.debug_quiet) RCLCPP_INFO(get_logger(),
                    "FlameRos/publishing = %4.1fms\n",
                    stats_.timings("publishing"));

  /*==================== Publish debug stuff ====================*/
  stats_.tick("debug_publishing");

  std_msgs::msg::Header hdr;
  hdr.stamp.sec = time;
  hdr.stamp.nanosec = 0;
  hdr.frame_id = live_frame_id_;

  if (params_.debug_draw_wireframe) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg =
        cv_bridge::CvImage(hdr, "bgr8",
                            sensor_->getDebugImageWireframe()).toImageMsg();
    debug_wireframe_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_features) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg =
        cv_bridge::CvImage(hdr, "bgr8",
                            sensor_->getDebugImageFeatures()).toImageMsg();
    debug_features_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_detections) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg =
        cv_bridge::CvImage(hdr, "bgr8",
                            sensor_->getDebugImageDetections()).toImageMsg();
    debug_detections_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_matches) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg =
        cv_bridge::CvImage(hdr, "bgr8",
                            sensor_->getDebugImageMatches()).toImageMsg();
    debug_matches_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_normals) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg =
        cv_bridge::CvImage(hdr, "bgr8",
                            sensor_->getDebugImageNormals()).toImageMsg();
    debug_normals_pub_.publish(debug_img_msg);
  }

  if (params_.debug_draw_idepthmap) {
    sensor_msgs::msg::Image::ConstSharedPtr debug_img_msg =
        cv_bridge::CvImage(hdr, "bgr8",
                            sensor_->getDebugImageInverseDepthMap()).toImageMsg();
    debug_idepthmap_pub_.publish(debug_img_msg);
  }

  stats_.tock("debug_publishing");
  if(!params_.debug_quiet) RCLCPP_INFO(get_logger(),
                    "FlameRos/debug_publishing = %4.1fms\n",
                    stats_.timings("debug_publishing"));

  stats_.tock("process_frame");

  if(!params_.debug_quiet) RCLCPP_INFO(get_logger(),
                    "FlameRos/process_frame = %4.1fms\n",
                    stats_.timings("process_frame"));

  return;
}

  /**
   * \brief Main processing loop.
   */
// void FlameRos::main() {
//     // Wait until input is initialized.
//     if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameRos: Waiting for calibration...\n");

//     while (!inited_) {
//       std::this_thread::yield();
//     }

    // Kinv_ = K_.inverse();

    // // Initialize depth sensor.
    // if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameRos: Constructing Flame...\n");
    // sensor_ = std::make_shared<flame::Flame>(width_,
    //                                          height_,
    //                                          K_,
    //                                          Kinv_,
    //                                          params_);

    // /*==================== Enter main loop ====================*/
    // if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameRos: Done. We are GO for launch!\n");
    
    //unsigned int frame_count = 0;

//     while (rclcpp::ok()) {
//       stats_.tick("main");

//       // Wait for queue to have items.
//       stats_.set("queue_size", queue().size());
//       stats_.tick("waiting");
//       std::unique_lock<std::recursive_mutex> lock(queue().mutex());
//       queue().non_empty().wait(lock, [this](){
//           return (queue().size() > 0);
//         });
//       lock.unlock();
//       stats_.tock("waiting");
//       if(!params_.debug_quiet) RCLCPP_INFO(get_logger(),
//           "FlameRos/waiting = %4.1fms, queue_size = %i\n",
//           stats_.timings("waiting"),
//           static_cast<int>(stats_.stats("queue_size")));

//       // Grab the first item in the queue.
//       Frame frame = queue().front();
//       queue().pop();
//       if ((pfs_inited_) && (num_imgs_ % subsample_factor_ == 0)) {
//         // Eat data.
//         // processFrame(frame.id, frame.time, Sophus::SE3f(frame.quat, frame.trans),
//         //              frame.img);

//         processFrame(frame_count++, frame.cam_frame_id, frame.time, Sophus::SE3f(frame.quat, frame.trans),
//                      frame.img);
//       }

//       /*==================== Timing stuff ====================*/
//       // Compute two measures of throughput in Hz. The first is the actual number of
//       // frames per second, the second is the theoretical maximum fps based on the
//       // runtime. They are not necessarily the same - the former takes external
//       // latencies into account.

//       // Compute maximum fps based on runtime.
//       double fps_max = 0.0f;
//       if (stats_.stats("fps_max") <= 0.0f) {
//         fps_max = 1000.0f / stats_.timings("main");
//       } else {
//         fps_max = 1.0f / (0.99 * 1.0f/stats_.stats("fps_max") +
//                           0.01 * stats_.timings("main")/1000.0f);
//       }
//       stats_.set("fps_max", fps_max);

//       // Compute actual fps (overall throughput of system).
//       stats_.tock("fps");
//       double fps = 0.0;
//       if (stats_.stats("fps") <= 0.0f) {
//         fps = 1000.0f / stats_.timings("fps");
//       } else {
//         fps = 1.0f / (0.99 * 1.0f/stats_.stats("fps") +
//                       0.01 * stats_.timings("fps")/1000.0f);
//       }
//       stats_.set("fps", fps);
//       stats_.tick("fps");

// #ifdef FLAME_WITH_FLA
//       last_update_sec_ = get_clock()->now().seconds();
// #endif

//       stats_.tock("main");

//       if ((num_imgs_ % load_integration_factor_) == 0) {
//         // Compute load stats.
//         fu::Load max_load, sys_load, pid_load;
//         load_.get(&max_load, &sys_load, &pid_load);
//         stats_.set("max_load_cpu", max_load.cpu);
//         stats_.set("max_load_mem", max_load.mem);
//         stats_.set("max_load_swap", max_load.swap);
//         stats_.set("sys_load_cpu", sys_load.cpu);
//         stats_.set("sys_load_mem", sys_load.mem);
//         stats_.set("sys_load_swap", sys_load.swap);
//         stats_.set("pid_load_cpu", pid_load.cpu);
//         stats_.set("pid_load_mem", pid_load.mem);
//         stats_.set("pid_load_swap", pid_load.swap);
//         stats_.set("pid", getpid());
//       }

//       // publishFlameNodeletStats(nodelet_stats_pub_,
//       //                          frame.id, frame.time,
//       //                          stats_.stats(), stats_.timings());

//       if(!params_.debug_quiet) RCLCPP_INFO(get_logger(),
//         "FlameRos/main(%i/%u) = %4.1fms/%.1fHz (%.1fHz)\n",
//         num_imgs_, frame.id, stats_.timings("main"),
//         stats_.stats("fps_max"), stats_.stats("fps"));

//       num_imgs_++;
//     }

//     return;
// }

} // namespace flame_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(flame_ros::FlameRos)
