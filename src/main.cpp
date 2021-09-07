#include <my_particle_filter/particle_filter.h>
#include <my_particle_filter/measurement_model.h>
#include <cstdlib>
#include <random>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pf");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS *my_costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", buffer);

  //particle_filter::ParticleFilter *pf = new particle_filter::ParticleFilter(costmap_ros, distance_costmap_ros);

  //pf->run_filter_();

  //particle_filter::Costmap_  *cm = new particle_filter::Costmap_(buffer);

  //cm->update_costmap_bounds();

  particle_filter::MeasurementModel *measurement_model = new particle_filter::MeasurementModel(my_costmap_ros);

  ros::spin();

  return 0;
}