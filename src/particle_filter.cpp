#include <my_particle_filter/particle_filter.h>
  #include <cstdlib>
#include <random>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>

using namespace std;




namespace particle_filter
{

  ParticleFilter::ParticleFilter(costmap_2d::Costmap2DROS *costmap_ros) : nh_{"particle_filter"}
  {

    ROS_INFO("Inside the ParticleFilter constructor! \n");

    my_costmap_ros = costmap_ros;
    costmap_ros_ = my_costmap_ros->getCostmap();

    size_x = costmap_ros_->getSizeInCellsX();
    size_y = costmap_ros_->getSizeInCellsY();

    num_particles = 100;

    update_map_bounds();

    particle_list_.resize(0);
    weight_list_.resize(0);

    
    marker_id_cnt = 0;

    //Class Publishers
    particle_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_pose", 10000, true);
    goal_marker_pub = nh_.advertise<visualization_msgs::Marker>("goal_markers", 10000);
    marker_array_pub = nh_.advertise<visualization_msgs::MarkerArray>("marker_array_", 10000);
    
    ros::NodeHandle nh_;
    ros::NodeHandle dummy_nh_("");

    first_run = true;
    
    //Subscribers
    odom_sub = dummy_nh_.subscribe("husky_velocity_controller/odom", 100, &ParticleFilter::odom_callback, this);
    initial_pose_sub = dummy_nh_.subscribe("initialpose", 100, &ParticleFilter::initial_pose_callback, this);
    laserscan_sub = dummy_nh_.subscribe("scan", 100, &ParticleFilter::laserscan_callback, this);

    //flags
    laserscan_flag = false;

    
    ROS_INFO("Sleeping for 2 seconds!\n");

    ros::Duration(2.0).sleep();

    initialize_particles_vector();
    
    laserscan_flag = false;
    
  }

  void ParticleFilter::laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg){


        if (!laserscan_flag){
        
            return;
        
        }

        publish_particle_list_(particle_list_);
    
        measurement_model = new particle_filter::MeasurementModel(my_costmap_ros, particle_list_);

        int num_beams_ = measurement_model->num_beams;

        double sensor_ang_inc = msg->angle_increment; 

        double target_ang_inc = (msg->angle_max - msg->angle_min)/(num_beams_ -1);
        
        int mul = target_ang_inc / sensor_ang_inc;

        ROS_INFO("mul: %d\n", mul);

        vector<pair<double, double> > Z_;

        for(int j = 0 ; j < msg->ranges.size(); j+= mul){

            double range_ = msg->ranges[j];
            double ang_ = msg->angle_min + j* sensor_ang_inc;
            
            ROS_INFO("ang_: %f\n", ang_);

            Z_.push_back({range_, ang_});

        }

        ROS_INFO("Z_.size(): %d\n", Z_.size());

        int flag_ = ((int)Z_.size() == num_beams_);

        if(!flag_){
          
          ROS_INFO("Something is wrong --- Z_.size() != num_beams_ \n");

        }

        for(int i =0 ;i < (int)Z_.size(); i++){
          
          ROS_INFO("Z_[%d]: %f\n", i, Z_[i]);

        }

        measurement_model->run_measurement_model(Z_);
        particle_list_ = measurement_model->get_particles();

        /*int np_ = particle_list_.size();
        ROS_INFO("np_: %d\n", np_);

        for(int i = 0 ;i < np_; i++){

            double x_ = particle_list_[i].pose.position.x, y_ = particle_list_[i].pose.position.y;
            
            tf2::Quaternion q_ = {particle_list_[i].pose.orientation.x, particle_list_[i].pose.orientation.y, particle_list_[i].pose.orientation.z, particle_list_[i].pose.orientation.w};
            double theta_ = get_yaw_from_quaternion(q_);

            ROS_INFO("x_: %f y_: %f theta_: %f\n", x_, y_, theta_);

        }
        
        
        //particle_list_ = measurement_model->get_particles();
        
        np_ = particles_.size();
        ROS_INFO("np_: %d\n", np_);

        for(int i = 0 ;i < np_; i++){

            double x_ = particles_[i].pose.position.x, y_ = particles_[i].pose.position.y;

            tf2::Quaternion q_ = {particles_[i].pose.orientation.x, particles_[i].pose.orientation.y, particles_[i].pose.orientation.z, particles_[i].pose.orientation.w};
            double theta_ = get_yaw_from_quaternion(q_);

            ROS_INFO("x_: %f y_: %f theta_: %f\n", x_, y_, theta_);

        }
        */
        
        laserscan_flag = false;
        
    }


  void ParticleFilter::publish_marker_array(const vector<pair<__uint32_t, __uint32_t>> &point_marker_array)
  {

    visualization_msgs::MarkerArray marker_array;

    for (int i = 0; i < (int)point_marker_array.size(); i++)
    {

      visualization_msgs::Marker marker;

      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();

      marker.ns = nh_.getNamespace();
      marker.id = marker_id_cnt++;

      marker.type = visualization_msgs::Marker::SPHERE;

      marker.action = visualization_msgs::Marker::ADD;

      double wx_, wy_;
      __uint32_t mx_ = point_marker_array[i].first, my_ = point_marker_array[i].second;

      costmap_ros_->mapToWorld(mx_, my_, wx_, wy_);

      marker.pose.position.x = wx_;
      marker.pose.position.y = wy_;
      marker.pose.position.z = 0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      marker_array.markers.push_back(marker);
    }

    marker_array_pub.publish(marker_array);
  }


  void ParticleFilter::delete_all_markers()
  {
    
    visualization_msgs::MarkerArray marker_array;

  
      visualization_msgs::Marker marker;

      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();

      marker.ns = nh_.getNamespace();
      marker.id = marker_id_cnt++;

      marker.type = visualization_msgs::Marker::SPHERE;

      marker.action = visualization_msgs::Marker::DELETEALL;

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      marker_array.markers.push_back(marker);
    
    //goal_marker_pub.publish(marker);
    marker_array_pub.publish(marker_array);
    
  }

  void ParticleFilter::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
  {

    /*ROS_INFO("Subscribed to the initialpose topic! \n");

    delete_all_markers();

    double wx_ = msg->pose.pose.position.x, wy_ = msg->pose.pose.position.y;

    ROS_INFO("frame: %s", msg->header.frame_id);

    tf2::Quaternion q_ = {msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    q_.normalize();

    ROS_INFO("wx_: %f wy_: %f\n", wx_, wy_);
    ROS_INFO("q_x: %f q_y: %f q_z: %f q_w: %f\n", q_[0], q_[1], q_[2], q_[3]);

    float yaw_ = get_yaw_from_quaternion(q_);

    ROS_INFO("yaw_: %f\n", yaw_);

    __uint32_t mx_, my_;

    ROS_INFO("costmap_frame_id: %s\n", my_costmap_ros->getGlobalFrameID());
    costmap_ros_->worldToMap((double)wx_, (double)wy_, mx_, my_);

    ROS_INFO("costmap_2d::NO_INFORMATION: %lu\n", costmap_2d::NO_INFORMATION);
    ROS_INFO("costmap_2d::LETHAL_OBSTACLE: %lu\n", costmap_2d::LETHAL_OBSTACLE);
    ROS_INFO("costmap_2d::FREE_SPACE: %lu\n", costmap_2d::FREE_SPACE);

    ROS_INFO("wx_: %f wy_: %f mx_: %lu my_: %lu\n", wx_, wy_, mx_, my_);

    ROS_INFO("cell_cost[%lu][%lu]: %lu\n", mx_, my_, costmap_ros_->getCost(mx_, my_));

    
    geometry_msgs::PoseStamped particle_pose_;

    particle_pose_.header = msg->header;
    particle_pose_.pose = msg->pose.pose;

    init_pose_ = particle_pose_;

    ROS_INFO("wx_ %f wy_: %f mx_: %lu my_: %lu \n", wx_, wy_, mx_, my_);

    double delta_x = curr_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    double delta_y = curr_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;

    ROS_INFO("old_delta_x: %f old_delta_y: %f", delta_x, delta_y);

    add_gaussian_noise(delta_x, linear_cov[0]);
    add_gaussian_noise(delta_y, linear_cov[1]);


    run_filter_();


    laserscan_flag = true;
  
    */
  
    laserscan_flag =  true;

  
  }

  float ParticleFilter::get_yaw_from_quaternion(tf2::Quaternion &q_)
  {

    float t3 = +2.0 * (q_[3] * q_[2] + q_[0] * q_[1]);
    float t4 = +1.0 - 2.0 * (q_[1] * q_[1] + q_[2] * q_[2]);

    float yaw_ = atan2(t3, t4);

    return yaw_;
  }

  void ParticleFilter::publish_marker(pair<__uint32_t, __uint32_t> point_)
  {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = nh_.getNamespace();
    marker.id = marker_id_cnt++;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    double wx_, wy_;
    __uint32_t mx_ = point_.first, my_ = point_.second;

    costmap_ros_->mapToWorld(mx_, my_, wx_, wy_);

    marker.pose.position.x = wx_;
    marker.pose.position.y = wy_;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    //ROS_INFO("goal_marker_pub.getNumSubscribers(): %lu\n", goal_marker_pub.getNumSubscribers());

    while (goal_marker_pub.getNumSubscribers() <= 0)
    {

      ROS_INFO("Subscriber not found --- sleeping for 1 second! \n");
      //goal_marker_pub.publish(marker);
      ros::Duration(1.0).sleep();
    }

    //ROS_INFO("Subscriber found\n");
    goal_marker_pub.publish(marker);

    //}
  }

  void ParticleFilter::odom_callback(const nav_msgs::OdometryConstPtr &msg)
  {

    curr_odom_ = *msg;

    if (first_run)
    {

      prev_odom_ = curr_odom_;
      first_run = false;
    }

    else
    {

      double dx_ = curr_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
      double dy_ = curr_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;

      double dis_ = dx_ * dx_ + dy_ * dy_;

      if (dis_ > 1)
      {

        ROS_INFO("Calling the odom callback! \n");

        ROS_INFO("Bot has moved 1 meter in odom frame prev_x: %d prev_y: %d curr_x: %d curr_y: %d \n", prev_odom_.pose.pose.position.x, prev_odom_.pose.pose.position.y, curr_odom_.pose.pose.position.x, curr_odom_.pose.pose.position.y);

        delete_all_markers();

        motion_model = new MotionModel(prev_odom_, curr_odom_, particle_list_); 
        motion_model->perform_motion_model_update();
        particle_list_ = motion_model->get_particles_();
        
        publish_particle_list_(particle_list_);

        prev_odom_ = curr_odom_;
      
      }
    }
  }

  void ParticleFilter::publish_particle_list_(const vector<geometry_msgs::PoseStamped>&particle_list_)
  {

    ROS_INFO("Inside the publish_particle_list function! \n");

    geometry_msgs::PoseArray pose_array_;

    pose_array_.header.frame_id = "map";
    pose_array_.header.stamp = ros::Time().now();

    for (int i = 0; i < (int)particle_list_.size(); i++)
    {

      pose_array_.poses.push_back(particle_list_[i].pose);
    }

    particle_pose_array_pub_.publish(pose_array_);
  }

  void ParticleFilter::update_map_bounds()
  {

    __uint32_t x_mn, x_mx, y_mn, y_mx;
    x_mn = size_x, y_mn = size_y;

    x_mx = 0, y_mx = 0;

    for (__uint32_t i = 0; i < size_x; i++)
    {

      for (__uint32_t j = 0; j < size_y; j++)
      {

        unsigned cell_cost = costmap_ros_->getCost(i, j);

        if (cell_cost != costmap_2d::NO_INFORMATION)
        {

          x_mn = min(x_mn, i);
          x_mx = max(x_mx, i);
          y_mn = min(y_mn, j);
          y_mx = max(y_mx, j);
        }
      }
    }

    map_xi = x_mn, map_xf = x_mx;
    map_yi = y_mn, map_yf = y_mx;

    ROS_INFO("x_mn: %d x_mx: %d \n", x_mn, x_mx);
    ROS_INFO("y_mn %d y_mx: %d \n", y_mn, y_mx);

    ROS_INFO("map_xi: %d map_xf: %d \n", map_xi, map_xf);
    ROS_INFO("map_yi: %d map_yf: %d \n", map_yi, map_yf);
  }

  void ParticleFilter::initialize_particles_vector()
  {

    particle_list_.resize(0);
    weight_list_.resize(0);
    normalized_weight_list.resize(0);

    ROS_INFO("Inside the initialize_particles_vector function!");

    for (int i = 0; i < num_particles; i++)
    {

      geometry_msgs::PoseStamped particle_;

      particle_.header.stamp = ros::Time().now();
      particle_.header.frame_id = "map";

      double wx_, wy_;

      int mx_, my_;

      mx_ = (map_xi + (rand() % (map_xf - map_xi)));
      my_ = (map_yi + (rand() % (map_yf - map_yi)));

      costmap_ros_->mapToWorld(mx_, my_, wx_, wy_);

      tf2::Quaternion quat_tf;
      ;
      double curr_ang = ((double)rand()) / ((double)RAND_MAX) * 6.28;
      quat_tf.setRPY(0, 0, curr_ang);

      particle_.pose.orientation.x = quat_tf[0];
      particle_.pose.orientation.y = quat_tf[1];
      particle_.pose.orientation.z = quat_tf[2];
      particle_.pose.orientation.w = quat_tf[3];

      particle_.pose.position.x = wx_;
      particle_.pose.position.y = wy_;

      particle_list_.push_back(particle_);
    }

  }
  
};
