#include <my_particle_filter/particle_filter.h>
#include <cstdlib>
#include <random>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

namespace particle_filter { 

    ParticleFilter::ParticleFilter(costmap_2d::Costmap2DROS* costmap_ros): nh_{"particle_filter"} {
        
        ROS_INFO("Inside the ParticleFilter constructor! \n");

        my_costmap_ros = costmap_ros; 
        costmap_ros_ = my_costmap_ros->getCostmap();

        size_x = costmap_ros_->getSizeInCellsX(); 
        size_y = costmap_ros_->getSizeInCellsY();

        num_particles = 100;
      
        update_map_bounds();

        particle_list_.resize(0);
        weight_list_.resize(0);

        //Class Publlishers 
        particle_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_pose",10000, true);
        
        //Class subscribers
        //odom_sub = nh_.subscribe("/husky_velocity_controller/odom", 1000, odom_call_back);

        ros::NodeHandle dummy_nh_("");
        
        first_run = true;
        odom_sub = dummy_nh_.subscribe("husky_velocity_controller/odom", 100, &ParticleFilter::odom_callback, this);


        //ros::spinOnce();

        //prev_odom_ = curr_odom_;

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
          
          //cout << "o.x : " << particle_.pose.orientation.x << " o.y: " << particle_.pose.orientation.y << " o.z: " << particle_.pose.orientation.z << " o.w: " << particle_.pose.orientation.w << endl;
          //ROS_INFO("o.x %d o.y: %d o.z: %d o.w: %d \n", particle_.pose.orientation.x, particle_.pose.orientation.y, particle_.pose.orientation.z, particle_.pose.orientation.w);
          
          particle_list_.push_back(particle_);

      }
  
  }

    double ParticleFilter::add_gaussian_noise(){
      
      std::default_random_engine generator;
      std::normal_distribution<double> distribution(5.0,2.0);
      
      const int nrolls=10000;  // number of experiments
      const int nstars=100
      int p[10]={};

        double number = distribution(generator);
        if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
      }

      std::cout << "normal_distribution (5.0,2.0):" << std::endl;

      for (int i=0; i<10; ++i) {
        std::cout << i << "-" << (i+1) << ": ";
        std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
      }

    }

    void ParticleFilter::perform_motion_model_update(){
      
      ROS_INFO("Inside the perform_motion_model_update function! \n");
      
      double delta_x = curr_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
      double delta_y = curr_odom_.pose.pose.position.y - curr_odom_.pose.pose.position.y;

      ROS_INFO("delta_x: %f delta_y: %f", delta_x, delta_y);

      tf2::Quaternion q_prev, q_curr, q_rot;
      
      q_prev[0] = prev_odom_.pose.pose.orientation.x; 
      q_prev[1] = prev_odom_.pose.pose.orientation.y; 
      q_prev[2] = prev_odom_.pose.pose.orientation.z; 
      q_prev[3] = prev_odom_.pose.pose.orientation.w;

      //ROS_INFO("q_prev.x: %f q_prev.y: %f q_prev.z: %f q_prev.w: %f \n", q_prev[0], q_prev[1], q_prev[2], q_prev[3]);

      q_curr[0] = curr_odom_.pose.pose.orientation.x; 
      q_curr[1] = curr_odom_.pose.pose.orientation.y;
      q_curr[2] = curr_odom_.pose.pose.orientation.z; 
      q_curr[3] = curr_odom_.pose.pose.orientation.w;

      //ROS_INFO("q_curr.x: %f q_curr.y: %f q_curr.z: %f q_curr.w: %f \n", q_curr[0], q_curr[1], q_curr[2], q_curr[3]);

      //ROS_INFO("q_prev_inv.x: %f q_prev_inv.y: %f q_prev_inv.z: %f q_prev_inv.w: %f \n", q_prev_inv[0], q_prev_inv[1], q_prev_inv[2], q_prev_inv[3]);

      q_rot = q_curr * (q_prev.inverse());
      q_rot = q_rot.normalized();

      //ROS_INFO("q_rot.x: %f q_rot.y: %f q_rot.z: %f q_rot.w: %f \n", q_rot[0], q_rot[1], q_rot[2], q_rot[3]);

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
        
        //ROS_INFO("q_new.x: %f q_new.y: %f q_new.z: %f q_new.w: %f \n", q_new[0], q_new[1], q_new[2], q_new[3]);
        
        q_new = q_new.normalized();      

        //ROS_INFO("q_new.x: %f q_new.y: %f q_new.z: %f q_new.w: %f \n", q_new[0], q_new[1], q_new[2], q_new[3]);


        geometry_msgs::PoseStamped particle_; 

        particle_.header.stamp = ros::Time().now();
        particle_.header.frame_id = "map";

        particle_.pose.position.x = new_x;
        particle_.pose.position.y = new_y;

        particle_.pose.orientation.x = q_new[0];
        particle_.pose.orientation.y = q_new[1]; 
        particle_.pose.orientation.z = q_new[2];
        particle_.pose.orientation.w = q_new[3];

        ROS_INFO("particle.o.x: %f particle.o.y: %f particle.o.z: %f particle.o.w: %f \n", particle_.pose.orientation.x , particle_.pose.orientation.y, particle_.pose.orientation.z, particle_.pose.orientation.w);

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