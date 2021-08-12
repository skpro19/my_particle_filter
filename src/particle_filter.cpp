#include <my_particle_filter/particle_filter.h>
#include <cstdlib>
#include <random>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace std;

namespace particle_filter { 

  ParticleFilter::ParticleFilter(costmap_2d::Costmap2DROS* costmap_ros): nh_{"particle_filter"} {
      
      ROS_INFO("Inside the ParticleFilter constructor! \n");

      my_costmap_ros = costmap_ros; 
      costmap_ros_ = my_costmap_ros->getCostmap();

      size_x = costmap_ros_->getSizeInCellsX(); 
      size_y = costmap_ros_->getSizeInCellsY();

      num_particles = 10;
    
      update_map_bounds();

      particle_list_.resize(0);
      weight_list_.resize(0);


      linear_cov = {0.001, 0.001};
      angular_cov = {0.001, 0.001, 0.03};

      //Class Publlishers 
      particle_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_pose",10000, true);
      
      ros::NodeHandle dummy_nh_("");
      
      first_run = true;

      odom_sub = dummy_nh_.subscribe("husky_velocity_controller/odom", 100, &ParticleFilter::odom_callback, this);
      initial_pose_sub = dummy_nh_.subscribe("initialpose", 100, &ParticleFilter::initial_pose_callback, this);
      

      //ros::spinOnce();

      //prev_odom_ = curr_odom_;

  }


  void ParticleFilter::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){

    ROS_INFO("Subscribed to the initialpose topic! \n");

    //float wx_ = msg->pose.pose.position.x, wy_ = msg->pose.pose.position.y;
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

    ROS_INFO("wx_: %f wy_: %f mx_: %lu my_: %lu\n",wx_, wy_, mx_, my_);

    ROS_INFO("cell_cost[%lu][%lu]: %lu\n", mx_, my_, costmap_ros_->getCost(mx_, my_));

    geometry_msgs::PoseStamped pose_;
    pose_.pose = msg->pose.pose;
    pose_.header = msg->header;

    ROS_INFO("------ Calling the ray_cast_particle function ---- \n");
    ray_cast_particle(pose_);

  }

  float ParticleFilter::get_yaw_from_quaternion(tf2::Quaternion &q_){

    float t3 = +2.0 * (q_[3] * q_[2] + q_[0] * q_[1]);
    float t4 = +1.0 - 2.0 * (q_[1] * q_[1] + q_[2] * q_[2]);

    float yaw_ = atan2(t3, t4);

    return yaw_;


  }

  void ParticleFilter::generate_fake_laserscan(const vector<pair<__uint32_t, __uint32_t> >&ray_casted_points, geometry_msgs::PoseStamped particle_){

    double ox_ = particle_.pose.position.x, oy_ = particle_.pose.position.y;

    vector<double> ray_casted_ranges;

    for(int i = 0; i <(int)ray_casted_points.size(); i++) {

      __uint32_t mx_ = ray_casted_points[i].first, my_ = ray_casted_points[i].second;

      double cx_, cy_;
      costmap_ros_->mapToWorld(mx_, my_, cx_, cy_);
      
      double dis_ = sqrt(((cx_ - ox_) * (cx_ - ox_)) + ((cy_ - oy_) * (cy_ - oy_)));
      
      ray_casted_ranges.push_back(dis_);

    }
    
    


  }

  void ParticleFilter::ray_cast_particle(geometry_msgs::PoseStamped &particle_){
    
    tf2::Quaternion q_ = {particle_.pose.orientation.x, particle_.pose.orientation.y, particle_.pose.orientation.z, particle_.pose.orientation.w};
    
    float theta_ = get_yaw_from_quaternion(q_);

    float ang_min = -2.356 + theta_;
    float ang_max = 2.356 + theta_;

    float ang_inc = 1.0/57;

    ROS_INFO("ang_inc: %f\n", ang_inc);

    float range_min = 0.1;
    float range_max = 30.0;

    double wx_ = particle_.pose.position.x, wy_ = particle_.pose.position.y;
    
    __uint32_t mx_, my_;

    costmap_ros_->worldToMap(wx_, wy_, mx_, my_);
    
    vector<pair<__uint32_t, __uint32_t> > ray_cast_ranges;

    for(float i = ang_min; i <= ang_max; i+=ang_inc) {

      int r = 0;

      int found = false;

      while(true){

        __uint32_t cx_ = mx_ + r * cos(i);
        __uint32_t cy_ = my_ + r * sin(i);

        if(cx_ > map_xf || cy_ > map_yf) {

          ray_cast_ranges.push_back({cx_, cy_});
          break;
        }

        if(costmap_ros_->getCost(cx_,cy_) == (int)costmap_2d::LETHAL_OBSTACLE) {
          
          ray_cast_ranges.push_back({cx_, cy_});
          //found = 1;
          break;
        }

        r++;

      }  
      
    
    }

    ROS_INFO("ray_cast_ranges.size(): %lu\n", (int)ray_cast_ranges.size());


  }


  void ParticleFilter::odom_callback(const nav_msgs::OdometryConstPtr &msg){

    //ROS_INFO("Odom data received");
    
    curr_odom_ = *msg;
    
    if(first_run) {


      prev_odom_ = curr_odom_;
      first_run = false;
    }

    else {

      double dx_ = curr_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x ; 
      double dy_ = curr_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;

      double dis_ = dx_ * dx_ + dy_ * dy_;

      if(dis_ > 1) {
        
        ROS_INFO("Bot has moved 1 meter in odom frame prev_x: %d prev_y: %d curr_x: %d curr_y: %d \n", prev_odom_.pose.pose.position.x , prev_odom_.pose.pose.position.y, curr_odom_.pose.pose.position.x, curr_odom_.pose.pose.position.y);
        perform_motion_model_update();
        prev_odom_ = curr_odom_;

      }

    }


    
  }



  void ParticleFilter::publish_particle_list_(){
    
    ROS_INFO("Inside the publish_particle_list function! \n");

    geometry_msgs::PoseArray pose_array_;

    pose_array_.header.frame_id = "map"; 
    pose_array_.header.stamp = ros::Time().now();

    for(int i =0 ; i < (int)particle_list_.size();i++) {

      pose_array_.poses.push_back(particle_list_[i].pose);
    
    }

    particle_pose_array_pub_.publish(pose_array_);

  }
  

  void ParticleFilter::run_filter_(){

    initialize_particles_vector();
    publish_particle_list_();
    
  }

  void ParticleFilter::update_map_bounds(){

  __uint32_t x_mn, x_mx, y_mn, y_mx;
  x_mn = size_x, y_mn = size_y;

  x_mx = 0 , y_mx = 0 ;

  for(__uint32_t i =0 ; i < size_x; i++) {

    for(__uint32_t j = 0 ; j < size_y; j++) {
      
      unsigned cell_cost = costmap_ros_->getCost(i, j);

      if(cell_cost != costmap_2d::NO_INFORMATION) {

        x_mn = min(x_mn, i);
        x_mx = max(x_mx, i); 
        y_mn = min(y_mn, j);
        y_mx = max(y_mx, j);

      }
      

    }

  }
  
  map_xi = x_mn, map_xf = x_mx; 
  map_yi = y_mn, map_yf = y_mx;

  ROS_INFO("x_mn: %d x_mx: %d \n" , x_mn, x_mx);
  ROS_INFO("y_mn %d y_mx: %d \n", y_mn, y_mx);

  ROS_INFO("map_xi: %d map_xf: %d \n" , map_xi , map_xf);
  ROS_INFO("map_yi: %d map_yf: %d \n", map_yi, map_yf);
  
}

  void ParticleFilter::initialize_particles_vector() {

    ROS_INFO("Inside the initialize_particles_vector function!");

    for(int i = 0; i < num_particles; i++) {

        geometry_msgs::PoseStamped particle_;

        particle_.header.stamp = ros::Time().now();
        particle_.header.frame_id = "map";
        
        double wx_, wy_;

        int mx_, my_;

        mx_ = (map_xi + (rand() % (map_xf - map_xi))) ;
        my_ = (map_yi + (rand() % (map_yf  - map_yi))) ;
        
        costmap_ros_->mapToWorld(mx_, my_, wx_, wy_);

        tf2::Quaternion quat_tf;;
        double curr_ang = ((double)rand()) / ((double)RAND_MAX) * 6.28;
        quat_tf.setRPY(0,0, curr_ang);
        
        particle_.pose.orientation.x = quat_tf[0]; 
        particle_.pose.orientation.y = quat_tf[1];
        particle_.pose.orientation.z = quat_tf[2];
        particle_.pose.orientation.w = quat_tf[3]; 

        particle_.pose.position.x = wx_;
        particle_.pose.position.y = wy_;
        
      
        particle_list_.push_back(particle_);

    }

}

  void ParticleFilter::add_gaussian_noise(double &point_, double variance_, double mean = 0){
    
    double sigma = sqrt(variance_);
  
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{mean,sigma};

    double noise = d(gen);
    
    point_ = point_ + noise;

    //return noise;

  }

  void ParticleFilter::add_gaussian_noise(tf2::Quaternion &q_t, vector<double> angular_cov, double mean = 0 ){
    
    vector<double> pose_v(3);

    for(int i = 0; i < 3; i++){

      double sigma = sqrt(angular_cov[i]);
      std::random_device rd{};
      std::mt19937 gen{rd()};
      std::normal_distribution<> d{mean,sigma};

      pose_v[i] = d(gen);
    
    }

    tf2::Quaternion quat_tf;;
    quat_tf.setRPY(pose_v[0],pose_v[1], pose_v[2]);
    
    q_t = q_t + quat_tf;

  }

  

  void ParticleFilter::perform_motion_model_update(){
    

    ROS_INFO("Inside the perform_motion_model_update function! \n");
    
    double delta_x = curr_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    double delta_y = curr_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y ;

    ROS_INFO("old_delta_x: %f old_delta_y: %f", delta_x, delta_y);

    add_gaussian_noise(delta_x, linear_cov[0]);
    add_gaussian_noise(delta_y, linear_cov[1]);

    //ROS_INFO("updated_delta_x: %f updated_delta_y: %f", delta_x, delta_y);


    tf2::Quaternion q_prev, q_curr, q_rot;
    
    q_prev[0] = prev_odom_.pose.pose.orientation.x; 
    q_prev[1] = prev_odom_.pose.pose.orientation.y; 
    q_prev[2] = prev_odom_.pose.pose.orientation.z; 
    q_prev[3] = prev_odom_.pose.pose.orientation.w;

    //ROS_INFO_STREAM("old_q_prev: " << q_prev);

    add_gaussian_noise(q_prev, angular_cov);

    //ROS_INFO_STREAM("new_q_prev: " << q_prev);


    q_curr[0] = curr_odom_.pose.pose.orientation.x; 
    q_curr[1] = curr_odom_.pose.pose.orientation.y;
    q_curr[2] = curr_odom_.pose.pose.orientation.z; 
    q_curr[3] = curr_odom_.pose.pose.orientation.w;

    //ROS_INFO_STREAM("old_q_curr: " << q_curr);

    add_gaussian_noise(q_curr, angular_cov);

    //ROS_INFO_STREAM("new_q_curr: " << q_curr);

    q_rot = q_curr * (q_prev.inverse());
    q_rot = q_rot.normalized();

    for(int i =0 ; i < (int)particle_list_.size();i++) {
      
      double prev_x, prev_y, new_x, new_y;

      prev_x = particle_list_[i].pose.position.x;
      prev_y = particle_list_[i].pose.position.y;

      new_x = prev_x + delta_x;
      new_y = prev_y + delta_y; 

      tf2::Quaternion q_old, q_new;
      q_old[0] = particle_list_[i].pose.orientation.x;
      q_old[1] = particle_list_[i].pose.orientation.y;
      q_old[2] = particle_list_[i].pose.orientation.z; 
      q_old[3] = particle_list_[i].pose.orientation.w;

      q_new = q_rot * q_old;
      
      q_new = q_new.normalized();      

      geometry_msgs::PoseStamped particle_; 

      particle_.header.stamp = ros::Time().now();
      particle_.header.frame_id = "map";

      particle_.pose.position.x = new_x;
      particle_.pose.position.y = new_y;

      particle_.pose.orientation.x = q_new[0];
      particle_.pose.orientation.y = q_new[1]; 
      particle_.pose.orientation.z = q_new[2];
      particle_.pose.orientation.w = q_new[3];

      //ROS_INFO("particle.o.x: %f particle.o.y: %f particle.o.z: %f particle.o.w: %f \n", particle_.pose.orientation.x , particle_.pose.orientation.y, particle_.pose.orientation.z, particle_.pose.orientation.w);

      particle_list_[i] = particle_;

      //particle_list_.push_back(particle_);

    }      
    
    publish_particle_list_();

  }



};


int main(int argc, char** argv){

  ros::init(argc, argv, "pf");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS* costmap_ros  = new costmap_2d::Costmap2DROS("global_costmap", buffer);  

  particle_filter::ParticleFilter* pf = new particle_filter::ParticleFilter(costmap_ros);

  pf->run_filter_();

  ros::spin();

  return 0;

}