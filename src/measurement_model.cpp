#include<my_particle_filter/measurement_model.h>

using namespace std;

namespace particle_filter{

    void MeasurementModel::initialize_model_params(const vector<int> &map_bounds){

        //map_bounds
        map_xi = map_bounds[0], map_xf = map_bounds[1];
        map_yi = map_bounds[2], map_yf = map_bounds[3];
        
        //fixed params 
        double z_max = 30; 

        double theta_sense = 0;
        double x_sense = 0.337, y_sense = 0 ; 
        
        double sigma_hit = 0.1;

        //variable params
        num_beams = 4;

    }   

    void MeasurementModel::run_measurement_model(const vector<geometry_msgs::PoseStamped> &particle_list){

        initialized_ = true;
        particles_ = particle_list;

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

            ROS_INFO("prob_ for %dth particle: %f\n", i, prob_);

        }

        initialized_ = false;
    
    }

    void MeasurementModel::initialize_subscribers(){
        
        ros::NodeHandle dummy_nh_("");
        laserscan_sub = dummy_nh_.subscribe("scan", 100, &MeasurementModel::laserscan_callback, this);
    
    }


    MeasurementModel::MeasurementModel(costmap_2d::Costmap2DROS* my_costmap_ros, const vector<int> &map_bounds){
        
        ROS_INFO("Inside the measurement model constructor!\n");

        costmap_ros = my_costmap_ros;
        costmap_ros_ = costmap_ros->getCostmap();

        initialize_model_params(map_bounds);

        initialize_subscribers();

        initialized_ = false;

    }



    double MeasurementModel::compute_prob_zero_centered_gaussian(double dist, double sd){

        double c = 1.0 / (sd *sqrt(2 * M_PI));
        double prob = c * exp((pow(dist,2))/(2 *pow(sd, 2)));
        return prob;

    }

    

    pair<double, double> MeasurementModel::get_closest_occupied_cell_from_(double x_k, double y_k) {

        __uint32_t mx_, my_;
        double wx_ = x_k, wy_ = y_k;

        costmap_ros_->worldToMap(wx_, wy_, mx_, my_);

        queue<pair<__uint32_t, __uint32_t> > q_;

        q_.push({mx_, my_});

        pair<__uint32_t, __uint32_t> target_cell_;

        bool found_ = false;

        while(!q_.empty()){

            pair<__uint32_t, __uint32_t> front_ = q_.front();
            q_.pop();

            int cx_ = (int)front_.first, cy_ = (int)front_.second;

            for(int i = cx_ - 1; i <= cx_ + 1; i++) {

                for(int j = cy_ - 1; j <= cy_ + 1; j++) {
                    
                    if(found_) {continue;}

                    if(i == (int)mx_ && j == (int)my_) {continue;}

                    if(i <= (int)map_xi || i >= (int)map_xf) {continue;}

                    if(j <= (int)map_yi || j >= (int)map_yf) {continue;}

                    unsigned int cost_ = costmap_ros_->getCost(i, j);

                    if(cost_ == costmap_2d::LETHAL_OBSTACLE){

                        found_ = 1; 
                        target_cell_ = {i,j}; 

                    }

                    else {

                        q_.push({i,j});

                    }
                }

            }

            if(found_) {break;}

        }

        if(!found_) {

            ROS_INFO("Something is wrong --- No nearby occupied cell found! \n");

        }

        double wxx_, wyy_;
        __uint32_t mxx_ = target_cell_.first , myy_ = target_cell_.second;

        costmap_ros_->mapToWorld(mxx_, myy_, wxx_ , wyy_);


        return {wxx_, wyy_};
    }

    double MeasurementModel::likelihood_field_range_finder_model(const vector<double> &Z_, const vector<double> &particle_pose_coords){

        ROS_INFO("Size of Z_: %d\n", Z_.size());

        int K = Z_.size(); 

        if(K != num_beams) {
            
            ROS_INFO("Something is wrong --- K(%d) != num_beams(%d)\n", K , num_beams);

        }

        double q = 1 ;
        
        double x = particle_pose_coords[0], y = particle_pose_coords[1], theta = particle_pose_coords[2];

        ROS_INFO("x: %f y: %f theta: %f \n", x , y, theta);

        for(int k = 1; k <= K ; k++) {
            
            double z_k = Z_[k];

            if(z_k != z_max) {

                double x_k  = x + (x_sense * cos(theta))  - (y_sense * sin(theta)) + (z_k * cos(theta + theta_sense));
                double y_k  = y + (y_sense * cos(theta))  - (x_sense * sin(theta)) + (z_k * sin(theta + theta_sense));

                pair<double, double> closest_occ_cell  = get_closest_occupied_cell_from_(x_k, y_k);

                double x_ = closest_occ_cell.first, y_ = closest_occ_cell.second;

                double dist_ = sqrt(( (x_k - x_) * (x_k - x_) ) + ((y_k - y_) * (y_k - y_)));

                q = q * compute_prob_zero_centered_gaussian(dist_, sigma_hit);

            }            

        }

        return q;
        

    }


};