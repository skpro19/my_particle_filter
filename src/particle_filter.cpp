#include <my_particle_filter/particle_filter.h>
#include <cstdlib>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;

namespace particle_filter {

    ParticleFilter::ParticleFilter(costmap_2d::Costmap2DROS* costmap_ros): nh_{"particle_filter"} {
        
        ROS_INFO("Inside the ParticleFilter constructor! \n");

        my_costmap_ros = costmap_ros; 
        costmap_ros_ = my_costmap_ros->getCostmap();

        size_x = costmap_ros_->getSizeInCellsX(); 
        size_y = costmap_ros_->getSizeInCellsY();

        num_particles = 1000;
      
        update_map_bounds();

        particle_list_.resize(0);
        weight_list_.resize(0);

        particle_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_pose",10000, true);
        
    }

    void ParticleFilter::publish_particle_list_(){
      
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
          cout << "curr_ang: " << curr_ang << endl;
          quat_tf.setRPY(0,0, curr_ang);

          geometry_msgs::Quaternion quat_msg; 
          
          tf2::convert(quat_msg, quat_tf);

          particle_.pose.orientation = quat_msg;
          particle_.pose.position.x = wx_;
          particle_.pose.position.y = wy_;

          particle_list_.push_back(particle_);

      }
  
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