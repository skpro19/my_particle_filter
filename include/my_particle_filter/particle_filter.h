#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>

namespace particle_filter {



    class ParticleFilter {
        
        
        public: 

            ParticleFilter(costmap_2d::Costmap2DROS* costmap_ros);
            void update_map_bounds();
            void update_particle_with_measurement_model();
            void initialize_particles_vector();
            void run_filter_();
            void publish_particle_list_();


        private:


            costmap_2d::Costmap2DROS* my_costmap_ros;
            int num_particles;
            costmap_2d::Costmap2D* costmap_ros_;
            __uint32_t size_x , size_y;
            ros::Publisher particle_pose_array_pub_;
            //ros::Publisher initial_pose_array_pub;
            ros::NodeHandle nh_;    
            int map_xi, map_xf, map_yi, map_yf;
            double res;
            std::vector<geometry_msgs::PoseStamped> particle_list_;
            std::vector<double>weight_list_;
            
    };


};