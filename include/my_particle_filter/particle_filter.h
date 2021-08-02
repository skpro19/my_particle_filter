#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>

namespace particle_filter {



    class ParticleFilter {
        
        public: 

            ParticleFilter(costmap_2d::Costmap2DROS* costmap_ros);
            void initialize_particle_cloud();
            void update_map_bounds();


        private:

            costmap_2d::Costmap2DROS* my_costmap_ros;
            costmap_2d::Costmap2D* costmap_ros_;
            int num_particles;
            __uint32_t size_x , size_y;
            std::vector<geometry_msgs::PoseStamped> particles;
            ros::Publisher initial_pose_pub;
            ros::Publisher initial_pose_array_pub;
            ros::NodeHandle nh_;    
            int map_xi, map_xf, map_yi, map_yf;
            double res;
        
    };


};