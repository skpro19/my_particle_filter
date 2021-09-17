#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <random>

#ifndef MOTION_MODEL_CPP
#define MOTION_MODEL__CPP


namespace particle_filter {


    class MotionModel {

        public: 

            MotionModel(const nav_msgs::Odometry &prev_pose, const nav_msgs::Odometry &curr_pose, const std::vector<geometry_msgs::PoseStamped> &particle_list_);
            void perform_motion_model_update();
            double get_yaw_from_quaternion(tf2::Quaternion &q_);
            double add_gaussian_noise(double x, double mean, double sigma);
            geometry_msgs::PoseStamped nav_odom_to_geometry_pose_stamped(const nav_msgs::Odometry &curr_odom_);
            std::vector<geometry_msgs::PoseStamped> get_particles_();

        private:

            geometry_msgs::PoseStamped prev_pose_, curr_pose_;
            std::vector<geometry_msgs::PoseStamped> particles_;
            double pos_mean, pos_std_dev, ori_mean, ori_std_dev;    
    
    };


};

#endif