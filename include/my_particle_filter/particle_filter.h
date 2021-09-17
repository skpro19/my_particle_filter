#include <my_particle_filter/measurement_model.h>
#include <my_particle_filter/motion_model.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>



#ifndef PARTICLE_FILTER_CPP
#define PARTICLE_FILTER_CPP


namespace particle_filter {



    class ParticleFilter {
        
        
        public: 

            ParticleFilter(costmap_2d::Costmap2DROS* costmap_ros);
            void update_map_bounds();
            void initialize_particles_vector();
            void publish_particle_list_(const std::vector<geometry_msgs::PoseStamped>&particle_list_);
            void odom_callback(const nav_msgs::OdometryConstPtr &msg);
            void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
            float get_yaw_from_quaternion(tf2::Quaternion &q_);
            void publish_marker(std::pair<__uint32_t, __uint32_t> point_);
            void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg);
            void delete_all_markers();
            void publish_marker_array(const std::vector<std::pair<__uint32_t, __uint32_t> >&point_marker_array);
            
        private:
            
            costmap_2d::Costmap2DROS* my_costmap_ros;
            int num_particles;
            costmap_2d::Costmap2D* costmap_ros_;
            __uint32_t size_x , size_y;
            ros::Publisher particle_pose_array_pub_,goal_marker_pub, marker_array_pub;
            
            ros::NodeHandle nh_;    
            int map_xi, map_xf, map_yi, map_yf;
            double res;
            
            std::vector<geometry_msgs::PoseStamped> particle_list_;
            std::vector<double>weight_list_, normalized_weight_list;
            
            ros::Subscriber odom_sub, initial_pose_sub, laserscan_sub;
            
            nav_msgs::Odometry curr_odom_, prev_odom_;
            bool first_run;
            
            int marker_id_cnt;

            //measurement model object
            MeasurementModel* measurement_model;

            //motion model object
            MotionModel* motion_model;      

            //ros::Publisher global_plan_pub, goal_marker_pub;
            //double x_cov, y_cov, qx_cov, qy_cov, qz_cov, qw_cov;
            
            std::vector<double>laserscan_ranges;
            bool laserscan_flag;
            
            
    };


};

#endif