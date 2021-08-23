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
            void publish_particle_list_(const std::vector<geometry_msgs::PoseStamped>&particle_list_);
            void perform_motion_model_update();
            void odom_callback(const nav_msgs::OdometryConstPtr &msg);
            void add_gaussian_noise(double &point_, double variance_, double mean);
            void add_gaussian_noise(tf2::Quaternion &q_t, std::vector<double> variance_, double mean);
            void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
            float get_yaw_from_quaternion(tf2::Quaternion &q_);
            void publish_laserscan(const std::vector<double>&ranges_, ros::Publisher &pub);
            void update_ray_cast_ranges(const geometry_msgs::PoseStamped &particle_pose_, const std::vector<std::pair<__uint32_t, __uint32_t> >&ray_cast_coords, std::vector<double> &ray_cast_ranges);
            void update_ray_cast_coords(geometry_msgs::PoseStamped &particle_, std::vector<std::pair<__uint32_t, __uint32_t> > &ray_cast_coords);
            void publish_marker(std::pair<__uint32_t, __uint32_t> point_);
            void update_laserscan_range(std::vector<double> &laserscan_range);
            void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg);
            double get_particle_weights(const std::vector<double> &ray_cast_ranges);
            void delete_all_markers();
            void publish_marker_array(const std::vector<std::pair<__uint32_t, __uint32_t> >&point_marker_array);
            double Gaussian(double mu, double sigma, double x);
            void publish_weighted_marker_array();
            void perform_measurement_update();
            void resample_weights();
            void publish_weighted_resampled_marker_array(const std::vector<geometry_msgs::PoseStamped>&resampled_particles_);
            geometry_msgs::PoseStamped return_noisy_particle(geometry_msgs::PoseStamped &particle_);
        
        private:
            
            costmap_2d::Costmap2DROS* my_costmap_ros;
            int num_particles;
            costmap_2d::Costmap2D* costmap_ros_;
            __uint32_t size_x , size_y;
            ros::Publisher particle_pose_array_pub_, fake_laser_pub, real_laser_pub,goal_marker_pub, marker_array_pub, weighted_marker_array_pub;
            ros::Publisher resampled_weighted_marker_array_pub;
            
            ros::NodeHandle nh_;    
            int map_xi, map_xf, map_yi, map_yf;
            double res;
            std::vector<geometry_msgs::PoseStamped> particle_list_;
            std::vector<double>weight_list_, normalized_weight_list;
            ros::Subscriber odom_sub, initial_pose_sub, laserscan_sub;
            nav_msgs::Odometry curr_odom_, prev_odom_;
            bool first_run;
            std::vector<double> linear_cov, angular_cov; 
            
            int marker_id_cnt;
            
            //ros::Publisher global_plan_pub, goal_marker_pub;
            //double x_cov, y_cov, qx_cov, qy_cov, qz_cov, qw_cov;
            
            std::vector<double>laserscan_ranges;
            bool laserscan_flag;
            
            //testing vars
            geometry_msgs::PoseStamped init_pose_;

            //scan params
            int num_readings_scan;
            double range_min_scan; ;
            double range_max_scan;;
            double ang_inc_scan;
            double ang_mn_scan;
            double ang_mx_scan;
            double laser_cov;


    };


};