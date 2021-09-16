#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <random>

#ifndef MEASUREMENT_MODEL_CPP
#define MEASUREMENT_MODEL_CPP


namespace particle_filter {



    class MeasurementModel {

        

        public: 

            MeasurementModel(costmap_2d::Costmap2DROS* my_costmap_ros,const std::vector<geometry_msgs::PoseStamped> &particles);
            double likelihood_field_range_finder_model(const std::vector<std::pair<double, double> > &Z_, const std::vector<double> &particle_pose_coords);
            std::pair<double, double> get_closest_occupied_cell_from_(double x_k, double y_k);
            double compute_prob_zero_centered_gaussian(double dist, double sd);
            void initialize_model_params();
            void run_measurement_model(const std::vector<std::pair<double, double> > &Z_);
            //void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg);
            void initialize_subscribers_and_publishers();
            double get_yaw_from_quaternion(tf2::Quaternion &q_);
            //void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
            void publish_marker(std::pair<__uint32_t, __uint32_t> point_);
            std::vector<geometry_msgs::PoseStamped> resampled_particles(const std::vector<double> &normalized_weights_);    
            std::vector<double> normalize_particle_weights(const std::vector<double> &weights_);
            void publish_particle_list_(const std::vector<geometry_msgs::PoseStamped>&particle_list_);
            void set_particles(const std::vector<geometry_msgs::PoseStamped> &particle_list_);
            std::vector<geometry_msgs::PoseStamped> get_particles();
            void add_noise_to_resampled_particles(std::vector<geometry_msgs::PoseStamped> &resampled_particles_);

            int num_beams;
            
        private:

            costmap_2d::Costmap2DROS *costmap_ros;
            costmap_2d::Costmap2D* costmap_ros_;
            double z_max; 
            double theta_sense, x_sense, y_sense; 
            double sigma_hit;

            int map_xi, map_xf; 
            int map_yi, map_yf;
      
            std::vector<geometry_msgs::PoseStamped> particles_;
            ros::Subscriber laserscan_sub, initial_pose_sub;
            ros::Publisher goal_marker_pub,particle_pose_array_pub_;            
            int marker_id_cnt =0;
            std::vector<double> weights_; 
            
    };


};

#endif