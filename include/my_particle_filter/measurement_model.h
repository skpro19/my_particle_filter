#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>


#ifndef MEASUREMENT_MODEL_CPP
#define MEASUREMENT_MODEL_CPP


namespace particle_filter {



    class MeasurementModel {

        

        public: 

            MeasurementModel(costmap_2d::Costmap2DROS* my_costmap_ros, const std::vector<int> &map_bounds);
            double likelihood_field_range_finder_model(const std::vector<double> &Z_, const std::vector<double> &particle_pose_coords);
            std::pair<double, double> get_closest_occupied_cell_from_(double x_k, double y_k);
            double compute_prob_zero_centered_gaussian(double dist, double sd);
            void initialize_model_params(const std::vector<int> &map_bounds);
            void run_measurement_model(const std::vector<geometry_msgs::PoseStamped> &particles_);
            void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg);
            void initialize_subscribers();
            double get_yaw_from_quaternion(tf2::Quaternion &q_);


        private:

            int num_beams;
            costmap_2d::Costmap2DROS *costmap_ros;
            costmap_2d::Costmap2D* costmap_ros_;
            double z_max; 
            double theta_sense, x_sense, y_sense; 
            double sigma_hit;

            int map_xi, map_xf; 
            int map_yi, map_yf;

            bool initialized_;
            
            std::vector<geometry_msgs::PoseStamped> particles_;
            ros::Subscriber laserscan_sub;
            

    };


};

#endif