#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

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
            //void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg);
            void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
            float get_yaw_from_quaternion(tf2::Quaternion &q_);
            //void generate_fake_laserscan(const std::vector<std::pair<__uint32_t, __uint32_t> >&ray_casted_points, geometry_msgs::PoseStamped particle_);
            void generate_fake_laserscan(const std::vector<double>&ranges_, const geometry_msgs::PoseStamped &particle_pose);
            void update_ray_cast_ranges(const geometry_msgs::PoseStamped &particle_pose_, const std::vector<std::pair<__uint32_t, __uint32_t> >&ray_cast_coords, std::vector<double> &ray_cast_ranges);
            void update_ray_cast_coords(geometry_msgs::PoseStamped &particle_, std::vector<std::pair<__uint32_t, __uint32_t> > &ray_cast_coords);
            void publish_marker(std::pair<__uint32_t, __uint32_t> point_);
            
        
        
        private:


            costmap_2d::Costmap2DROS* my_costmap_ros;
            int num_particles;
            costmap_2d::Costmap2D* costmap_ros_;
            __uint32_t size_x , size_y;
            ros::Publisher particle_pose_array_pub_, fake_laser_pub,goal_marker_pub;
            ros::NodeHandle nh_;    
            int map_xi, map_xf, map_yi, map_yf;
            double res;
            std::vector<geometry_msgs::PoseStamped> particle_list_;
            std::vector<double>weight_list_;
            ros::Subscriber odom_sub, initial_pose_sub;
            nav_msgs::Odometry curr_odom_, prev_odom_;
            bool first_run;
            std::vector<double> linear_cov, angular_cov; 
            int marker_id_cnt;
            //ros::Publisher global_plan_pub, goal_marker_pub;
            //double x_cov, y_cov, qx_cov, qy_cov, qz_cov, qw_cov;

            
    };


};