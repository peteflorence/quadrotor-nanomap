#include "motion.h"
#include "kd_tree.h"

#include "nanomap/nanomap.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>
#include <chrono>
#include <algorithm> 

class DepthImageCollisionEvaluator {
public:
	DepthImageCollisionEvaluator() {
		//K << 304.8, 0.0, 160.06, 0.0, 304.8, 119.85, 0.0, 0.0, 1.0;
                K << 308.57684326171875, 0.0, 154.6868438720703, 0.0, 308.57684326171875, 120.21442413330078, 0.0, 0.0, 1.0;
                K /= binning;
                K(2,2) = 1.0;

                R << 1,0,0,0,1,0,0,0,1;
	}
	
  void UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new);
  void UpdateLaserPointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new);
  void UpdateRotationMatrix(Matrix3 const R);
  void UpdateBodyToRdf(Matrix3 const& R);

  bool computeDeterministicCollisionOnePositionKDTree(Vector3 const& robot_position);

  bool IsBehind(Vector3 robot_position);
  bool IsOutsideDeadBand(Vector3 robot_position);
  double IsOutsideFOV(Vector3 robot_position);
  double AddOutsideFOVPenalty(Vector3 robot_position, double probability_of_collision);
  
  double computeProbabilityOfCollisionNPositionsKDTree_DepthImage(Vector3 const& robot_position, Vector3 const& sigma_robot_position, bool early_exit);
  double computeProbabilityOfCollisionNPositionsKDTree_Laser(Vector3 const& robot_position, Vector3 const& sigma_robot_position);
  double computeProbabilityOfCollisionNPositionsKDTree(Vector3 const& robot_position, Vector3 const& sigma_robot_position, std::vector<pcl::PointXYZ> const& closest_pts, bool interpolate);

  void setCameraInfo(double bin, double width, double height, Matrix3 K_camera_info);

  NanoMap nanomap;

private:

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_laser_cloud_ptr;

  Vector3 sigma_depth_point = Vector3(0.01, 0.01, 0.01);

  Matrix3 K;
  double binning = 4.0;
  double num_x_pixels = 320/binning;
  double num_y_pixels = 240/binning;

  KDTree<double> my_kd_tree_depth_image;
  KDTree<double> my_kd_tree_laser;

  Matrix3 R; //rotation matrix from ortho_body frame into camera rdf frame
  Matrix3 R_body_to_rdf;
  Matrix3 R_body_to_rdf_inverse;

  double p_collision_behind = 0.5;
  double p_collision_left_right_fov = 0.5;
  double p_collision_up_down_fov = 0.5;
  double p_collision_occluded = 0.999;
  double p_collision_beyond = 0.5;

};
