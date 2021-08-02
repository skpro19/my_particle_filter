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


        num_particles = 100;
        res = 0.05;


        update_map_bounds();

        initial_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("initial_poses", 1000);
        initial_pose_array_pub = nh_.advertise<geometry_msgs::PoseArray>("initial_pose_array", 1000);

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

    void ParticleFilter::initialize_particle_cloud(){
        
        geometry_msgs::PoseArray initial_pose_array;

        initial_pose_array.header.stamp = ros::Time().now();
        initial_pose_array.header.frame_id = "map";
        
        for(int i = 0; i < num_particles; i++) {

            geometry_msgs::Pose curr_pose;

            //curr_pose.header.frame_id = my_costmap_ros->getGlobalFrameID();
            //curr_pose.header.frame_id = "map";
            //curr_pose.header.stamp = ros::Time().now();

            double wx_, wy_;

            int mx_, my_;

            mx_ = (map_xi + (rand() % (map_xf - map_xi))) ;
            my_ = (map_yi + (rand() % (map_yf  - map_yi))) ;
            
            costmap_ros_->mapToWorld(mx_, my_, wx_, wy_);

            tf2::Quaternion quat_tf;;
            double curr_ang = ((double)rand()) / ((double)RAND_MAX) * 6.28;
            quat_tf.setRPY(0,0, curr_ang);

            geometry_msgs::Quaternion quat_msg; 
            
            tf2::convert(quat_msg, quat_tf);

            curr_pose.orientation = quat_msg;
            curr_pose.position.x = wx_; 
            curr_pose.position.y= wy_;

            //particles.push_back(curr_pose);
            initial_pose_array.poses.push_back(curr_pose);
            //cout << "curr_pose.x: " << curr_pose.pose.position.x << " curr_pose.y: " << curr_pose.pose.position.y << endl;
            
            
        }
        
        while(true){
        
            initial_pose_array_pub.publish(initial_pose_array);
            ros::Duration(5.0).sleep();
        
        }
    
    }


   
   


};


int main(int argc, char** argv){

    ros::init(argc, argv, "pf");

    
        tf2_ros::Buffer buffer(ros::Duration(10));
        tf2_ros::TransformListener tf(buffer);

        cout << "H1 one!" << endl;   
        costmap_2d::Costmap2DROS* costmap_ros  = new costmap_2d::Costmap2DROS("global_costmap", buffer);  
        cout << "H2 two!" << endl;
    

   particle_filter::ParticleFilter* pf = new particle_filter::ParticleFilter(costmap_ros);
   pf->initialize_particle_cloud();


    ros::spin();

    return 0;

}