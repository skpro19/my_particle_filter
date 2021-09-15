#include<my_particle_filter/measurement_model.h>

using namespace std;

namespace particle_filter{

    
    MeasurementModel::MeasurementModel(costmap_2d::Costmap2DROS* my_costmap_ros, const vector<int> &map_bounds, const vector<geometry_msgs::PoseStamped> &particle_list_){
        
        ROS_INFO("Inside the measurement model constructor!\n");

        costmap_ros = my_costmap_ros;
        costmap_ros_ = costmap_ros->getCostmap();

        initialize_model_params(map_bounds);

        initialize_subscribers_and_publishers();

        particles_ = particle_list_;

        initialized_ = false;

    }
    
    void MeasurementModel::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){

        initialized_ = true;

        ROS_INFO("initial_pose_callback called!\n");

        double wx_ = msg->pose.pose.position.x, wy_ = msg->pose.pose.position.y;

        string frame = msg->header.frame_id;

        ROS_INFO("frame: %s\n", frame);

        __uint32_t mx_ , my_;
        costmap_ros_->worldToMap(wx_, wy_, mx_, my_);

        ROS_INFO("mx_: %lu my_: %lu\n", mx_, my_);

        pair<double, double> closest_cell_ = get_closest_occupied_cell_from_(wx_, wy_);

        ROS_INFO("closest_cell.x: %f closest_cell.y: %f\n", closest_cell_.first, closest_cell_.second);

        __uint32_t cx_, cy_;
        costmap_ros_->worldToMap(closest_cell_.first, closest_cell_.second, cx_, cy_);
        
        ROS_INFO("cx_: %d cy_: %d\n", cx_, cy_);


    }


    void MeasurementModel::initialize_model_params(const vector<int> &map_bounds){

        //map_bounds
        map_xi = map_bounds[0], map_xf = map_bounds[1];
        map_yi = map_bounds[2], map_yf = map_bounds[3];
        
        double wx_i, wx_f, wy_i, wy_f;
        costmap_ros_->mapToWorld(map_xi, map_yi, wx_i, wy_i);
        costmap_ros_->mapToWorld(map_xf, map_yf, wx_f, wy_f);

        ROS_INFO("map_xi: %d map_xf: %d\n", map_xi, map_xf);
        ROS_INFO("map_yi: %d map_yf: %d\n", map_yi, map_yf);
        ROS_INFO("wx_i: %f wx_f: %f\n", wx_i, wx_f);
        ROS_INFO("wy_i: %f wy_f: %f\n", wy_i, wy_f);

        //fixed params 
        z_max = 20; 

        theta_sense = 0;
        x_sense = 0.337, y_sense = 0 ; 
        
        sigma_hit = 2;

        //variable params
        num_beams = 4;

    }   

    vector<geometry_msgs::PoseStamped> MeasurementModel::resample_weights(const vector<double> &normalized_weights_){
        
        ROS_INFO("Inside the resample_weights function!\n");

        int num_particles = particles_.size();

        vector<double> cum_weights;
        vector<int> resampled_particle_list;
        vector<geometry_msgs::PoseStamped> resampled_particles_;

        cum_weights.resize(num_particles);

        cum_weights[0] = normalized_weights_[0];

        for (int i = 1; i < num_particles; i++){

            cum_weights[i] = cum_weights[i - 1] + normalized_weights_[i];
        
        }

        double r = (rand() / double(RAND_MAX)) * (1.0 / (1.0 * num_particles));

        ROS_INFO("num_particles: %d\n", num_particles);
        ROS_INFO("r: %f\n", r);

        int flag_ = (num_particles == weights_.size());

        if (!flag_) ROS_INFO("ALERT --- num_particles  != weight_list.size()\n");

        int counter = 0;

        set<int> unique_particles;

        for (int i = 0; i < num_particles; i++){

            double r_ = r + (i * (1.0 / (1.0 * num_particles)));

            while (r_ > cum_weights[counter]){

                counter++;
            }

            resampled_particle_list.push_back(counter);

            resampled_particles_.push_back(particles_[counter]);

            unique_particles.insert(counter);
            
        }

        
        ROS_INFO("unique_particles: %d\n", unique_particles.size());
        ROS_INFO("resampled_particles_list.size(): %d\n", resampled_particle_list.size());

        ROS_INFO("Printing the index of the resampled particles!\n");
        
        for(int  i =0 ; i <num_particles; i++) {
        
            ROS_INFO("%d ", resampled_particle_list[i]);
        
        }

        return resampled_particles_;

     }


    void MeasurementModel::run_measurement_model(){

        initialized_ = false;
        //particles_ = particle_list;

    }

    double MeasurementModel::get_yaw_from_quaternion(tf2::Quaternion &q_){

        float t3 = +2.0 * (q_[3] * q_[2] + q_[0] * q_[1]);
        float t4 = +1.0 - 2.0 * (q_[1] * q_[1] + q_[2] * q_[2]);

        float yaw_ = atan2(t3, t4);

        return yaw_;
        
    }


    void MeasurementModel::laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg){
        
        
        if (!initialized_){
        
            return;
        
        }
  
        weights_.resize(0);
        
        double sensor_ang_inc = msg->angle_increment; 

        double target_ang_inc = (msg->angle_max - msg->angle_min)/(num_beams -1);
        
        int mul = target_ang_inc / sensor_ang_inc;

        ROS_INFO("mul: %d\n", mul);

        for(int i =0 ; i < (int)particles_.size(); i++) {

            vector<double> Z_;

            for(int j = 0 ; j < msg->ranges.size(); j+= mul){

                Z_.push_back(msg->ranges[j]);

            }

            tf2::Quaternion q_ = {particles_[i].pose.orientation.x, particles_[i].pose.orientation.y, particles_[i].pose.orientation.z, particles_[i].pose.orientation.w};
            double theta_ = get_yaw_from_quaternion(q_);

            vector<double> particle_pose_coords = {particles_[i].pose.position.x, particles_[i].pose.position.y, theta_};
            
            double prob_ = likelihood_field_range_finder_model(Z_, particle_pose_coords);

            weights_.push_back(prob_);

            ROS_INFO("prob_ for %dth particle: %f\n", i, prob_);

        }

        vector<double> normalized_weights_ = normalize_particle_weights(weights_);

        vector<geometry_msgs::PoseStamped> resampled_particles_ = resample_weights(normalized_weights_);

        ROS_INFO("resmapled_particles_.size(): %d\n", resampled_particles_.size());

        int num_particles = (int)particles_.size();

        for(int i =0 ; i < num_particles; i++){

            ROS_INFO("weights_[%d]: %f normalized_weights_[%d]: %f\n", i,  weights_[i], i, normalized_weights_[i]);

        }
        
        publish_particle_list_(resampled_particles_);

        initialized_ = false;
    
    }


    void MeasurementModel::publish_particle_list_(const vector<geometry_msgs::PoseStamped>&particle_list_){

        ROS_INFO("Inside the publish_particle_list function! \n");

        geometry_msgs::PoseArray pose_array_;

        pose_array_.header.frame_id = "map";
        pose_array_.header.stamp = ros::Time().now();

        for (int i = 0; i < (int)particle_list_.size(); i++){

            pose_array_.poses.push_back(particle_list_[i].pose);
        
        }

        particle_pose_array_pub_.publish(pose_array_);
    
    }

    vector<double> MeasurementModel::normalize_particle_weights(const vector<double> &weights_){

        int num_particles = (int)particles_.size();

        vector<double> normalized_weights_;
        normalized_weights_.resize(0);
        normalized_weights_.resize(num_particles);

        double sum_ = 0 ;

        for(int i =0 ; i < num_particles; i++) {

            sum_ = sum_ + weights_[i];

        }

        ROS_INFO("sum_: %f\n", sum_);

        for(int i =0 ; i < num_particles; i++) {

            normalized_weights_[i] = weights_[i]/sum_;

        }

        return normalized_weights_;

    }

    void MeasurementModel::initialize_subscribers_and_publishers(){
        
        ros::NodeHandle dummy_nh_("");
        laserscan_sub = dummy_nh_.subscribe("scan", 100, &MeasurementModel::laserscan_callback, this);
        initial_pose_sub = dummy_nh_.subscribe("initialpose", 100, &MeasurementModel::initial_pose_callback, this);

        goal_marker_pub = dummy_nh_.advertise<visualization_msgs::Marker>("goal_markers", 10000);
        particle_pose_array_pub_ = dummy_nh_.advertise<geometry_msgs::PoseArray>("particle_pose", 10000, true);
    
    }


    void MeasurementModel::publish_marker(pair<__uint32_t, __uint32_t> point_){

        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();


        marker.ns = "";
        marker.id = marker_id_cnt++;

        marker.type = visualization_msgs::Marker::CUBE;

        marker.action = visualization_msgs::Marker::ADD;

        double wx_, wy_;
        __uint32_t mx_ = point_.first, my_ = point_.second;

        costmap_ros_->mapToWorld(mx_, my_, wx_, wy_);

        marker.pose.position.x = wx_;
        marker.pose.position.y = wy_;
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(10.0);

        
        /*while (goal_marker_pub.getNumSubscribers() <= 0)
        {

        ROS_INFO("Subscriber not found --- sleeping for 1 second! \n");
        //goal_marker_pub.publish(marker);
        ros::Duration(1.0).sleep();
        }

        //ROS_INFO("Subscriber found\n");
        */
       
       goal_marker_pub.publish(marker);

    }

    

    double MeasurementModel::compute_prob_zero_centered_gaussian(double dist, double sd){

        double c = 1.0 / (sd *sqrt(2 * M_PI));
        
        double prob = c * exp((-pow(dist,2))/(2 *pow(sd, 2)));
        
        ROS_INFO("c: %f prob: %f \n", c, prob);

        return prob;

    }

    pair<double, double> MeasurementModel::get_closest_occupied_cell_from_(double x_k, double y_k) {
        
        __uint32_t mx_, my_;
        costmap_ros_->worldToMap(x_k, y_k, mx_, my_);

        set<pair<__uint32_t, __uint32_t> > visited_;

        queue<pair<__uint32_t, __uint32_t> > q_;
        q_.push({mx_, my_});

        int found_ = 0;

        pair<__uint32_t, __uint32_t> target_cell_;

        while(!q_.empty()){
            
            pair<__uint32_t, __uint32_t> front_ = q_.front();
            
            q_.pop();

            __uint32_t cx_ = front_.first, cy_=  front_.second;

            //ROS_INFO("cx_: %d cy_: %d q_.size(): %d\n", cx_, cy_, q_.size());

            if(visited_.find({cx_, cy_}) != visited_.end()) {continue;}

            visited_.insert(front_);

            for(int i = (int)cx_ - 1; i <= ((int)cx_ + 1) && !found_; i++) {

                for(int j  = (int)cy_ - 1; j<= ((int)cy_ + 1) && !found_; j++) {
                    
                    //if(i < map_xi || i > map_xf) {continue;}

                    //if(j < map_yi || j > map_yf) {continue;}

                    if(visited_.find({i,j}) != visited_.end()) {continue;}

                    unsigned int cost_ = costmap_ros_->getCost(i, j);

                    if(cost_ == (__uint32_t)costmap_2d::LETHAL_OBSTACLE) {
                        
                        found_ = 1;
                        target_cell_ = {i, j};
                    
                    }

                    if(!found_){

                        q_.push({i,j});

                    }

                }

            }

            if(found_){

                double tx_, ty_;
                costmap_ros_->mapToWorld(target_cell_.first, target_cell_.second, tx_, ty_);
                return {tx_, ty_};

            }
            
        }

        if(!found_) {

            ROS_INFO("Something is wrong --- no nearby obstacle found!--- \n");
            return {-1, -1};
        }



    }


    
    double MeasurementModel::likelihood_field_range_finder_model(const vector<double> &Z_, const vector<double> &particle_pose_coords){

        ROS_INFO("Size of Z_: %d\n", Z_.size());

        int K = Z_.size(); 

        if(K != num_beams) {
            
            ROS_INFO("Something is wrong --- K(%d) != num_beams(%d)\n", K , num_beams);

        }

        double q = 1 ;
        
        double x = particle_pose_coords[0], y = particle_pose_coords[1], theta = particle_pose_coords[2];

        ROS_INFO("x: %f y: %f theta: %f\n", x, y, theta);

        for(int k = 1; k <= K ; k++) {
            
            double z_k = Z_[k];

            if(z_k < z_max) {
                
                //ROS_INFO("k: %d z_k: %f\n", k , z_k);

                //double x_k  = (x_sense * cos(theta));
                //double y_k  =  (y_sense * cos(theta));

                //double x_k = z_k * cos(theta + theta_sense);
                //double y_k = z_k * sin(theta + theta_sense);

                double x_k  = x + (x_sense * cos(theta))  - (y_sense * sin(theta)) + (z_k * cos(theta + theta_sense));
                double y_k  = y + (y_sense * cos(theta))  - (x_sense * sin(theta)) + (z_k * sin(theta + theta_sense));
                
                __uint32_t mx_k, my_k ;
                costmap_ros_->worldToMap(x_k, y_k, mx_k, my_k);
                
                publish_marker({mx_k, my_k});

                //ROS_INFO("x_k: %f y_k: %f theta: %f\n", x_k, y_k, theta);

                pair<double, double> closest_occ_cell  = get_closest_occupied_cell_from_(x_k, y_k);

                double x_ = closest_occ_cell.first, y_ = closest_occ_cell.second;
                
                //ROS_INFO("x_: %f y_: %f theta: %f\n", x_, y_, theta);

                double dist_ = sqrt(( (x_k - x_) * (x_k - x_) ) + ((y_k - y_) * (y_k - y_)));

                //ROS_INFO("dist_: %f\n", dist_);

                q = q * compute_prob_zero_centered_gaussian(dist_, sigma_hit);

                //ROS_INFO("q: %f\n", q);

            }            

        }

        return q;
        

    }


};