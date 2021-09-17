#include <my_particle_filter/motion_model.h>

using namespace std;

namespace particle_filter{


    MotionModel::MotionModel(const nav_msgs::Odometry &prev_pose, const nav_msgs::Odometry &curr_pose, const vector<geometry_msgs::PoseStamped> &particle_list_){

        prev_pose_ = nav_odom_to_geometry_pose_stamped(prev_pose);
        curr_pose_ = nav_odom_to_geometry_pose_stamped(curr_pose);
        
        particles_ = particle_list_;

        pos_mean = 0, ori_mean = 0 ;

        pos_std_dev = 0.5, ori_std_dev = acos(-1)/15.0;
    
    }

    geometry_msgs::PoseStamped MotionModel::nav_odom_to_geometry_pose_stamped(const nav_msgs::Odometry &curr_odom_){

        geometry_msgs::PoseStamped pose_;
        
        pose_.header.frame_id = "map"; 
        pose_.header.stamp = ros::Time::now();

        pose_.pose.position.x = curr_odom_.pose.pose.position.x; 
        pose_.pose.position.y = curr_odom_.pose.pose.position.y;

        pose_.pose.orientation.x = curr_odom_.pose.pose.orientation.x; 
        pose_.pose.orientation.y = curr_odom_.pose.pose.orientation.y; 
        pose_.pose.orientation.z = curr_odom_.pose.pose.orientation.z; 
        pose_.pose.orientation.w = curr_odom_.pose.pose.orientation.w; 
         
        return pose_;

    }

    double MotionModel::get_yaw_from_quaternion(tf2::Quaternion &q_){

        float t3 = +2.0 * (q_[3] * q_[2] + q_[0] * q_[1]);
        float t4 = +1.0 - 2.0 * (q_[1] * q_[1] + q_[2] * q_[2]);

        float yaw_ = atan2(t3, t4);

        return yaw_;
    }

    void MotionModel::perform_motion_model_update(){

        double x_i = prev_pose_.pose.position.x , y_i = prev_pose_.pose.position.y; 
        double x_f = curr_pose_.pose.position.x , y_f = curr_pose_.pose.position.y;

        tf2::Quaternion q_i = {prev_pose_.pose.orientation.x, prev_pose_.pose.orientation.y,prev_pose_.pose.orientation.z,prev_pose_.pose.orientation.w};
        tf2::Quaternion q_f = {curr_pose_.pose.orientation.x, curr_pose_.pose.orientation.y,curr_pose_.pose.orientation.z,curr_pose_.pose.orientation.w };

        int num_particles = (int)particles_.size();

        double theta_i = get_yaw_from_quaternion(q_i), theta_f = get_yaw_from_quaternion(q_f);

        double del_x = x_f - x_i; 
        double del_y = y_f - y_i; 
        double del_theta = theta_f - theta_i;

        std::default_random_engine pos_gen_, ori_gen;

        std::normal_distribution<double> pos_dist_(pos_mean, pos_std_dev), ori_dist_(ori_mean, ori_std_dev);

        vector<geometry_msgs::PoseStamped> updated_particles_;
        updated_particles_.resize(0);

        for(int i = 0; i < num_particles; i++) {

            double dx_ = del_x + pos_dist_(pos_gen_);
            double dy_ = del_y + pos_dist_(pos_gen_);

            double dtheta = del_theta + ori_dist_(ori_gen);
            
            double x_ = particles_[i].pose.position.x, y_ = particles_[i].pose.position.y;

            tf2::Quaternion q_ = {particles_[i].pose.orientation.x, particles_[i].pose.orientation.y,particles_[i].pose.orientation.z,particles_[i].pose.orientation.w};
            double theta_ = get_yaw_from_quaternion(q_);
            
            x_ = x_ + dx_;
            y_ = y_ + dy_;
            theta_ = theta_ + dtheta;
            
            q_.setRPY(0, 0, theta_);
            q_.normalize();

            geometry_msgs::PoseStamped updated_particle_ = particles_[i];
            updated_particle_.pose.position.x = x_; 
            updated_particle_.pose.position.y = y_; 
            
            updated_particle_.pose.orientation.x = q_[0]; 
            updated_particle_.pose.orientation.y = q_[1]; 
            updated_particle_.pose.orientation.z= q_[2]; 
            updated_particle_.pose.orientation.w = q_[3]; 
            
            updated_particles_.push_back(updated_particle_);
        
        }
        
        particles_ = updated_particles_;
    
    }

    vector<geometry_msgs::PoseStamped> MotionModel::get_particles_(){

        return particles_;

    }




};