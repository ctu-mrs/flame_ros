#include <flame_ros/flame_ros.hpp>

#define NODE_NAME "flame"

namespace flame_ros {

FlameRos::FlameRos(const rclcpp::NodeOptions & options) : rclcpp::Node(NODE_NAME, options) {
    // std::signal(SIGSEGV, crash_handler);
    // std::signal(SIGILL, crash_handler);
    // std::signal(SIGABRT, crash_handler);
    // std::signal(SIGFPE, crash_handler);

    mrs_lib::ParamLoader param_loader(shared_from_this(), NODE_NAME);

    load_ = std::move(fu::LoadTracker(getpid()));

    num_imgs_ = 0;

    // Setup tf.
    //tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    /*==================== Input Params ====================*/
    param_loader.loadParam("input/camera_frame_id", camera_frame_id_);
    param_loader.loadParam("input/camera_world_frame_id", camera_world_frame_id_);
    param_loader.loadParam("input/subsample_factor", subsample_factor_);
    param_loader.loadParam("input/poseframe_subsample_factor", poseframe_subsample_factor_);
    param_loader.loadParam("input/use_poseframe_updates", use_poseframe_updates_);
    param_loader.loadParam("input/poseframe_child_frame_id", poseframe_child_frame_id_);
    param_loader.loadParam("input/resize_factor", resize_factor_);

    /*==================== Output Params ====================*/
    param_loader.loadParam("output/quiet", params_.debug_quiet);
    param_loader.loadParam("output/mesh", publish_mesh_);
    param_loader.loadParam("output/idepthmap", publish_idepthmap_);
    param_loader.loadParam("output/depthmap", publish_depthmap_);
    param_loader.loadParam("output/cloud", publish_cloud_);
    param_loader.loadParam("output/features", publish_features_);
    param_loader.loadParam("output/stats", publish_stats_);
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

#ifdef FLAME_WITH_FLA
    bool use_camera_info = false;
    param_loader.loadParam("input/use_camera_info", &use_camera_info);

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
      param_loader.loadParam("/samros/camera/image_width", &width);
      param_loader.loadParam("/samros/camera/image_height", &height);

      if (((width != 640) && (width != 1280)) ||
          ((height != 512) && (height != 1024))) {
        ROS_ERROR("FlameNodelet: Unexpected image size = (%i, % i)\n",
                  width, height);
      }

      double fx, fy, cx, cy;
      param_loader.loadParam("/samros/camera/intrinsics/fu", &fx);
      param_loader.loadParam("/samros/camera/intrinsics/fv", &fy);
      param_loader.loadParam("/samros/camera/intrinsics/pu", &cx);
      param_loader.loadParam("/samros/camera/intrinsics/pv", &cy);

      double k1, k2, p1, p2, k3;
      param_loader.loadParam("/samros/camera/distortion/k1", &k1);
      param_loader.loadParam("/samros/camera/distortion/k2", &k2);
      param_loader.loadParam("/samros/camera/distortion/p1", &p1);
      param_loader.loadParam("/samros/camera/distortion/p2", &p2);
      param_loader.loadParam("/samros/camera/distortion/k3", &k3);

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
    param_loader.loadParam("fla/node_id", &tmp);
    node_id_ = tmp;

    param_loader.loadParam("fla/heart_beat_dt", &heart_beat_dt_);
    param_loader.loadParam("fla/alarm_timeout", &alarm_timeout_);
    param_loader.loadParam("fla/fail_timeout", &fail_timeout_);

    heart_beat_ = nh.createTimer(ros::Duration(heart_beat_dt_),
                                 &FlameNodelet::heartBeat, this);
    heart_beat_pub_ = nh.advertise<fla_msgs::ProcessStatus>("/globalstatus", 1);
#else
    // Image resizing not supported for non-FLA.
    FLAME_ASSERT(resize_factor_ == 1);

    // Setup input stream.
    input_ = std::make_shared<ros_sensor_streams::
                              TrackedImageStream>(camera_world_frame_id_, shared_from_this());
#endif

    pfs_inited_ = true; // Default values.
    first_pf_id_ = 0;
    if (use_poseframe_updates_) {
        // Wait until first pf message received with >= 2 pfs so that we can
        // estimate the poseframe subsample factor and offset.
        pfs_inited_ = false;

        // Subscribe to poseframe topic.
        //poseframe_sub_ = nh.subscribe("poseframes", 1,
        //                              &FlameNodelet::poseframeCallback, this);
        mrs_lib::SubscriberHandlerOptions shopts(shared_from_this());
        shopts.node_name = NODE_NAME;
        //shopts.no_message_timeout = no_message_timeout;
        poseframe_sub_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Path>(
            shopts,
            "poseframes",
            &FlameRos::poseframeCallback
        );
    }

    // Set up publishers. For some reason this appears to take a while.
    if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameNodelet: Setting up publishers...\n");

    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    if (publish_idepthmap_) {
      idepth_pub_ = it_->advertiseCamera("idepth_registered/image_rect", 5);
    }
    if (publish_depthmap_) {
      depth_pub_ = it_->advertiseCamera("depth_registered/image_rect", 5);
    }
    if (publish_features_) {
      features_pub_ = it_->advertiseCamera("depth_registered_raw/image_rect", 5);
    }
    if (publish_mesh_) {
      mesh_pub_ = mrs_lib::PublisherHandler<pcl_msgs::msg::PolygonMesh>(shared_from_this(), "mesh");
    }
    if (publish_cloud_) {
      //cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 5);
      cloud_pub_ = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(shared_from_this(), "cloud");
    }
    // if (publish_stats_) {
    //   //stats_pub_ = nh.advertise<FlameStats>("stats", 5);
    //   stats_pub_ = mrs_lib::PublisherHandler<FlameStats>(shared_from_this(), "stats");
    //   //nodelet_stats_pub_ = nh.advertise<FlameNodeletStats>("nodelet_stats", 5);
    //   nodelet_stats_pub_ = mrs_lib::PublisherHandler<FlameNodeletStats>(shared_from_this(), "nodelet_stats");
    // }

    if (params_.debug_draw_wireframe) {
      debug_wireframe_pub_ = it_->advertise("debug/wireframe", 1);
    }
    if (params_.debug_draw_features) {
      debug_features_pub_ = it_->advertise("debug/features", 1);
    }
    if (params_.debug_draw_detections) {
      debug_detections_pub_ = it_->advertise("debug/detections", 1);
    }
    if (params_.debug_draw_matches) {
      debug_matches_pub_ = it_->advertise("debug/matches", 1);
    }
    if (params_.debug_draw_normals) {
      debug_normals_pub_ = it_->advertise("debug/normals", 1);
    }
    if (params_.debug_draw_idepthmap) {
      debug_idepthmap_pub_ = it_->advertise("debug/idepthmap", 1);
    }

    // Kick off main thread.
    thread_ = std::thread(&FlameRos::main, this);

    RCLCPP_INFO(get_logger(), "flame_ros constructed");
}

  /**
   * @brief Callback for receiving poseframe poses.
   */
void FlameRos::poseframeCallback(const nav_msgs::msg::Path::ConstSharedPtr& msg) {
    if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameNodelet: Got a poseframe message!\n");

    // Get transform to camera_world.
    geometry_msgs::TransformStamped tf;
    try {
      // Need to remove leading "/" if it exists.
      std::string frame_id = msg->header.frame_id;
      if (frame_id[0] == '/') {
        frame_id = frame_id.substr(1, frame_id.size()-1);
      }
      tf = tf_buffer_.lookupTransform(camera_world_frame_id_, frame_id,
                                      ros::Time(msg->header.stamp),
                                      ros::Duration(1.0/15));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
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
                                      ros::Time(msg->header.stamp),
                                      ros::Duration(1.0/15));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    Sophus::SE3f to_camera;
    ros_sensor_streams::tfToSophusSE3<float>(tf.transform, &to_camera);

    // Extract ids and poses.
    std::vector<uint32_t> pf_ids(msg->poses.size());
    std::vector<Sophus::SE3f> pf_poses(msg->poses.size());
    for (int ii = 0; ii < msg->poses.size(); ++ii) {
      pf_ids[ii] = msg->poses[ii].header.seq;

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
    }

    return;
}

  /**
   * \brief Main processing loop.
   */
void main() {
    // Wait until input is initialized.
    if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameNodelet: Waiting on calibration...\n");

    while (!input_->inited()) {
      std::this_thread::yield();
    }

    Kinv_ = input_->K().inverse();

    // Initialize depth sensor.
    if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameNodelet: Constructing Flame...\n");
    sensor_ = std::make_shared<flame::Flame>(input_->width(),
                                             input_->height(),
                                             input_->K(),
                                             Kinv_,
                                             params_);

    /*==================== Enter main loop ====================*/
    if(!params_.debug_quiet) RCLCPP_INFO(get_logger(), "FlameNodelet: Done. We are GO for launch!\n");
    
    while (ros::ok()) {
      stats_.tick("main");

      // Wait for queue to have items.
      stats_.set("queue_size", input_->queue().size());
      stats_.tick("waiting");
      std::unique_lock<std::recursive_mutex> lock(input_->queue().mutex());
      input_->queue().non_empty().wait(lock, [this](){
          return (input_->queue().size() > 0);
        });
      lock.unlock();
      stats_.tock("waiting");
      NODELET_INFO_COND(!params_.debug_quiet,
                        "FlameNodelet/waiting = %4.1fms, queue_size = %i\n",
                        stats_.timings("waiting"),
                        static_cast<int>(stats_.stats("queue_size")));

      // Grab the first item in the queue.
      Frame frame = input_->queue().front();
      input_->queue().pop();
      if ((pfs_inited_) && (num_imgs_ % subsample_factor_ == 0)) {
        // Eat data.
        processFrame(frame.id, frame.time, Sophus::SE3f(frame.quat, frame.trans),
                     frame.img);
      }

      /*==================== Timing stuff ====================*/
      // Compute two measures of throughput in Hz. The first is the actual number of
      // frames per second, the second is the theoretical maximum fps based on the
      // runtime. They are not necessarily the same - the former takes external
      // latencies into account.

      // Compute maximum fps based on runtime.
      double fps_max = 0.0f;
      if (stats_.stats("fps_max") <= 0.0f) {
        fps_max = 1000.0f / stats_.timings("main");
      } else {
        fps_max = 1.0f / (0.99 * 1.0f/stats_.stats("fps_max") +
                          0.01 * stats_.timings("main")/1000.0f);
      }
      stats_.set("fps_max", fps_max);

      // Compute actual fps (overall throughput of system).
      stats_.tock("fps");
      double fps = 0.0;
      if (stats_.stats("fps") <= 0.0f) {
        fps = 1000.0f / stats_.timings("fps");
      } else {
        fps = 1.0f / (0.99 * 1.0f/stats_.stats("fps") +
                      0.01 * stats_.timings("fps")/1000.0f);
      }
      stats_.set("fps", fps);
      stats_.tick("fps");

#ifdef FLAME_WITH_FLA
      last_update_sec_ = ros::Time::now().toSec();
#endif

      stats_.tock("main");

      if ((num_imgs_ % load_integration_factor_) == 0) {
        // Compute load stats.
        fu::Load max_load, sys_load, pid_load;
        load_.get(&max_load, &sys_load, &pid_load);
        stats_.set("max_load_cpu", max_load.cpu);
        stats_.set("max_load_mem", max_load.mem);
        stats_.set("max_load_swap", max_load.swap);
        stats_.set("sys_load_cpu", sys_load.cpu);
        stats_.set("sys_load_mem", sys_load.mem);
        stats_.set("sys_load_swap", sys_load.swap);
        stats_.set("pid_load_cpu", pid_load.cpu);
        stats_.set("pid_load_mem", pid_load.mem);
        stats_.set("pid_load_swap", pid_load.swap);
        stats_.set("pid", getpid());
      }

      publishFlameNodeletStats(nodelet_stats_pub_,
                               frame.id, frame.time,
                               stats_.stats(), stats_.timings());

      NODELET_INFO_COND(!params_.debug_quiet,
                        "FlameNodelet/main(%i/%u) = %4.1fms/%.1fHz (%.1fHz)\n",
                        num_imgs_, frame.id, stats_.timings("main"),
                        stats_.stats("fps_max"), stats_.stats("fps"));

      num_imgs_++;
    }

    return;
}

} // namespace flame_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(flame_ros::FlameRos)