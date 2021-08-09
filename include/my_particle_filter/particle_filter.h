#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace particle_filter {



    class ParticleFilter {
        
        
        public: 

            ParticleFilter(costmap_2d::Costmap2DROS* costmap_ros);
            void update_map_bounds();
            void update_particle_with_measurement_model();
            void initialize_particles_vector();
            void run_filter_();
            void publish_particle_list_();
            void perform_motion_model_update();
            void odom_callback(const nav_msgs::OdometryConstPtr &msg);
            void add_gaussian_noise(double &point_, double variance_, double mean);
            void add_gaussian_noise(tf2::Quaternion &q_t, std::vector<double> variance_, double mean);
    
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
            ros::Subscriber odom_sub;
            nav_msgs::Odometry curr_odom_, prev_odom_;
            bool first_run;
            std::vector<double> linear_cov, angular_cov; 

            //double x_cov, y_cov, qx_cov, qy_cov, qz_cov, qw_cov;

            
    };


};