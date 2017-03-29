#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include "fla_msgs/ProcessStatus.h"

#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"

#include <mutex>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <chrono>

#include "motion_selector.h"
#include "attitude_generator.h"
#include "motion_visualizer.h"

#include "fla_utils/param_utils.h"

class MotionSelectorNode {
public:

	MotionSelectorNode() : nh("~") {


		// Initialization
		double acceleration_interpolation_min;
		double soft_top_speed;
        double speed_at_acceleration_max;
        double acceleration_interpolation_max;

		fla_utils::SafeGetParam(nh, "soft_top_speed", soft_top_speed);
		fla_utils::SafeGetParam(nh, "acceleration_interpolation_min", acceleration_interpolation_min);
		fla_utils::SafeGetParam(nh, "yaw_on", yaw_on);
		fla_utils::SafeGetParam(nh, "use_depth_image", use_depth_image);
        fla_utils::SafeGetParam(nh, "speed_at_acceleration_max", speed_at_acceleration_max);
        fla_utils::SafeGetParam(nh, "acceleration_interpolation_max", acceleration_interpolation_max);
        fla_utils::SafeGetParam(nh, "flight_altitude", flight_altitude);
        fla_utils::SafeGetParam(nh, "use_3d_library", use_3d_library);
        fla_utils::SafeGetParam(nh, "max_e_stop_pitch_degrees", max_e_stop_pitch_degrees);
        fla_utils::SafeGetParam(nh, "laser_z_below_project_up", laser_z_below_project_up);
        fla_utils::SafeGetParam(nh, "A_dolphin", A_dolphin);
        fla_utils::SafeGetParam(nh, "T_dolphin", T_dolphin);

		this->soft_top_speed_max = soft_top_speed;

		motion_selector.InitializeLibrary(use_3d_library, final_time, soft_top_speed, acceleration_interpolation_min, speed_at_acceleration_max, acceleration_interpolation_max);
		motion_selector.SetNominalFlightAltitude(flight_altitude);
		attitude_generator.setZsetpoint(flight_altitude);

		motion_visualizer.initialize(&motion_selector, nh, &best_traj_index, final_time);
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
	    for(;;){
		    try {

		      tf_buffer_.lookupTransform("world", "body", 
		                                    ros::Time(0), ros::Duration(30.0));
		    } catch (tf2::TransformException &ex) {
		      continue;
		    }

		    break;
		}
		PublishOrthoBodyTransform(0.0, 0.0); // initializes ortho_body transform to be with 0, 0 roll, pitch
		srand ( time(NULL) ); //initialize the random seed

		// Subscribers

		pose_sub = nh.subscribe("pose_topic", 100, &MotionSelectorNode::OnPose, this);
		velocity_sub = nh.subscribe("twist_topic", 100, &MotionSelectorNode::OnVelocity, this);
        
		camera_info_sub = nh.subscribe("depth_camera_info_topic", 1, &MotionSelectorNode::OnCameraInfo, this);
		depth_image_sub = nh.subscribe("depth_camera_pointcloud_topic", 1, &MotionSelectorNode::OnDepthImage, this);
        
        max_speed_sub = nh.subscribe("/max_speed", 1, &MotionSelectorNode::OnMaxSpeed, this);
		local_goal_sub = nh.subscribe("local_goal_topic", 1, &MotionSelectorNode::OnLocalGoal, this);
		//value_grid_sub = nh.subscribe("/value_grid", 1, &MotionSelectorNode::OnValueGrid, this);
		laser_scan_sub = nh.subscribe("laser_scan_topic", 1, &MotionSelectorNode::OnScan, this);


		// Publishers
		carrot_pub = nh.advertise<visualization_msgs::Marker>( "carrot_marker_topic", 0 );
		gaussian_pub = nh.advertise<visualization_msgs::Marker>( "gaussian_visualization_topic", 0 );
		attitude_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("attitude_setpoint_topic", 1);
		attitude_setpoint_visualization_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_visualization_topic", 1);
		status_pub = nh.advertise<fla_msgs::ProcessStatus>("status_topic", 0);
 	}

	void OnMaxSpeed(const std_msgs::Float64 msg) {
		soft_top_speed_max = msg.data;
	}

	bool got_camera_info = false;
	void OnCameraInfo(const sensor_msgs::CameraInfo msg) {
		if (got_camera_info) {
			return;
		}
		double height = msg.height;
		double width = msg.width;
		Matrix3 K_camera_info;
		K_camera_info << msg.K[0], msg.K[1], msg.K[2], msg.K[3], msg.K[4], msg.K[5], msg.K[6], msg.K[7], msg.K[8];
		if (msg.binning_x != msg.binning_y) { std::cout << "WARNING: Binning isn't same for camera info" << std::endl;}
		double bin = msg.binning_x;
		DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
		if (depth_image_collision_ptr != nullptr) {
			depth_image_collision_ptr->setCameraInfo(bin, width, height, K_camera_info);
			got_camera_info = true;
			ROS_WARN_THROTTLE(1.0, "Received camera info");
			//std::cout << "K_camera_info " << K_camera_info << std::endl;
		}
		depth_sensor_frame = msg.header.frame_id;
	}

	void SetThrustForLibrary(double thrust) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
			motion_library_ptr->setThrust(thrust);
		}
	}

	geometry_msgs::TransformStamped GetTransformToWorld() {
		geometry_msgs::TransformStamped tf;
	    try {

	      tf = tf_buffer_.lookupTransform("world", "ortho_body", 
	                                    ros::Time(0), ros::Duration(1.0/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("ID 1 %s", ex.what());
	      return tf;
	    }
	    return tf;
	}

	bool CheckIfInevitableCollision(std::vector<double> const hokuyo_collision_probabilities) {
		for (size_t i = 0; i < hokuyo_collision_probabilities.size(); i++) {
			if (hokuyo_collision_probabilities.at(i) < 0.6) {
				return false;
			}
		}
		return true;
	}

	void ReactToSampledPointCloud() {
		if (!got_camera_info && use_depth_image) {
			ROS_WARN_THROTTLE(1.0, "Haven't received camera info yet");
		}

		auto t1 = std::chrono::high_resolution_clock::now();
		mutex.lock();
		motion_selector.computeBestEuclideanMotion(carrot_ortho_body_frame, best_traj_index, desired_acceleration);
		// geometry_msgs::TransformStamped tf = GetTransformToWorld();
		// motion_selector.computeBestDijkstraMotion(carrot_ortho_body_frame, carrot_world_frame, tf, best_traj_index, desired_acceleration);
	    mutex.unlock();
		
		mutex.lock();
      	std::vector<double> collision_probabilities = motion_selector.getCollisionProbabilities();
      	std::vector<double> hokuyo_collision_probabilities = motion_selector.getHokuyoCollisionProbabilities();
		motion_visualizer.setCollisionProbabilities(collision_probabilities);
		if (executing_e_stop || CheckIfInevitableCollision(hokuyo_collision_probabilities)) {
			ExecuteEStop();
		}
	    else if (yaw_on) {
	    	SetYawFromMotion();
	    } 
	    mutex.unlock();

		PublishCurrentAttitudeSetpoint();
	}

	void ExecuteEStop() {
		best_traj_index = 0; // this overwrites the "best acceleration motion"

		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		// If first time entering e stop, compute open loop parameters
		if (!executing_e_stop) {
			begin_e_stop_time = ros::Time::now().toSec();
			if (motion_library_ptr != nullptr) {
				double e_stop_acceleration_magnitude = 9.8*tan(max_e_stop_pitch_degrees * M_PI / 180.0);
				Vector3 initial_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.0);
				Vector3 e_stop_acceleration = -1.0 * e_stop_acceleration_magnitude * initial_velocity_ortho_body/initial_velocity_ortho_body.norm();
				motion_library_ptr->setBestAccelerationMotion(e_stop_acceleration);
				Vector3 end_jerk_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.2);

				e_stop_time_needed = end_jerk_velocity_ortho_body.norm() / e_stop_acceleration_magnitude / 0.85;
				ROS_WARN_THROTTLE(1.0, "E-STOPPING");
			}
		}
		executing_e_stop = true;
		if (motion_library_ptr != nullptr) {
			desired_acceleration = motion_library_ptr->getMotionFromIndex(best_traj_index).getAcceleration();
		}


		// Check if time to exit open loop e stop
		double e_stop_time_elapsed = ros::Time::now().toSec() - begin_e_stop_time;
		//std::cout << "E STOP TIME ELAPSED " << e_stop_time_elapsed << std::endl;
		if (e_stop_time_elapsed > e_stop_time_needed) {
			executing_e_stop = false;
		}
	}

	void ComputeBestAccelerationMotion() {
		if (executing_e_stop) { //Does not compute if executing e stop
			return;
		}

		mutex.lock();
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {

			// compute best acceleration in open field
			double time_to_eval = 0.5;
			Vector3 initial_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.0);
			Vector3 position_if_dont_accel = initial_velocity_ortho_body*time_to_eval;
			Vector3 vector_towards_goal = (carrot_ortho_body_frame - position_if_dont_accel);
			Vector3 best_acceleration = ((vector_towards_goal/vector_towards_goal.norm()) * soft_top_speed_max - initial_velocity_ortho_body) / time_to_eval;
			double current_max_acceleration = motion_library_ptr->getNewMaxAcceleration();
			if (best_acceleration.norm() > current_max_acceleration) {
				best_acceleration = best_acceleration * current_max_acceleration / best_acceleration.norm();
			}
			motion_library_ptr->setBestAccelerationMotion(best_acceleration);

			// if within stopping distance, line search for best stopping acceleration
			Vector3 stop_position = motion_library_ptr->getMotionFromIndex(0).getTerminalStopPosition(0.5);
			double stop_distance = stop_position.dot(vector_towards_goal/vector_towards_goal.norm());
			double distance_to_carrot = carrot_ortho_body_frame(0);
			
			int max_line_searches = 10;
			int counter_line_searches = 0;
			while ( (stop_distance > distance_to_carrot) && (counter_line_searches < max_line_searches) ) {
				best_acceleration = best_acceleration * distance_to_carrot / stop_distance;
				if (best_acceleration.norm() > current_max_acceleration) {
					best_acceleration = best_acceleration * current_max_acceleration / best_acceleration.norm();
				}
				motion_library_ptr->setBestAccelerationMotion(best_acceleration);
				stop_position = motion_library_ptr->getMotionFromIndex(0).getTerminalStopPosition(0.5);
				stop_distance = stop_position.dot(vector_towards_goal/vector_towards_goal.norm());
				counter_line_searches++;	
			} 

		}
		mutex.unlock();
	}

	void PublishCurrentAttitudeSetpoint() {
		mutex.lock();
		if (use_3d_library) {
			AltitudeFeedbackOnBestMotion();
		}
		Vector3 attitude_thrust_desired = attitude_generator.generateDesiredAttitudeThrust(desired_acceleration);
		SetThrustForLibrary(attitude_thrust_desired(2));
		mutex.unlock();
		PublishAttitudeSetpoint(attitude_thrust_desired);
	}

	void AltitudeFeedbackOnBestMotion() {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
				Motion best_motion = motion_library_ptr->getMotionFromIndex(best_traj_index);
				Vector3 best_motion_position_ortho_body =  best_motion.getPosition(0.5);
				Vector3 best_motion_position_world = TransformOrthoBodyToWorld(best_motion_position_ortho_body);
				double new_z_setpoint = best_motion_position_world(2);
				attitude_generator.setZsetpoint(new_z_setpoint);
		}
	}

	bool UseDepthImage() {
		return use_depth_image;
	}

	void drawAll() {
		mutex.lock();
		motion_visualizer.drawAll();
		mutex.unlock();
	}

private:


	void SetYawFromMotion() {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {

			// get position at t=0
			Vector3 initial_position_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getPosition(0.0);
			// get velocity at t=0
			Vector3 initial_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.5);
			Vector3 final_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.5);
			// normalize velocity
			double speed_initial = initial_velocity_ortho_body.norm();
			double speed_final = final_velocity_ortho_body.norm();
			if (speed_final != 0) {
				final_velocity_ortho_body = final_velocity_ortho_body / speed_final;
			}

			// add normalized velocity to position to get a future position
			Vector3 final_position_ortho_body = initial_position_ortho_body + initial_velocity_ortho_body;
			// yaw towards future position using below

			Vector3 final_position_world = TransformOrthoBodyToWorld(final_position_ortho_body);
			
			if (speed_initial < 2.0 && carrot_ortho_body_frame.norm() < 1.0) {
				motion_selector.SetSoftTopSpeed(soft_top_speed_max);
				return;
			}
			if ((final_position_world(0) - pose_global_x)!= 0) {
				double potential_bearing_azimuth_degrees = CalculateYawFromPosition(final_position_world);
				double actual_bearing_azimuth_degrees = -pose_global_yaw * 180.0/M_PI;
				double bearing_error = potential_bearing_azimuth_degrees - actual_bearing_azimuth_degrees;
				while(bearing_error > 180) { 
					bearing_error -= 360;
				}
				while(bearing_error < -180) { 
					bearing_error += 360;
				}

				if (abs(bearing_error) < 60.0)  {
					motion_selector.SetSoftTopSpeed(soft_top_speed_max);
					bearing_azimuth_degrees = potential_bearing_azimuth_degrees;
					return;
				}
				motion_selector.SetSoftTopSpeed(0.1);
				if (speed_initial < 0.5) {
					bearing_azimuth_degrees = CalculateYawFromPosition(carrot_world_frame);
				}
			}
		}
	}

	double CalculateYawFromPosition(Vector3 final_position) {
		return 180.0/M_PI*atan2(-(final_position(1) - pose_global_y), final_position(0) - pose_global_x);	
	}
	

	void SetGoalFromBearing() {
		bool go;
		nh.param("go", go, false);
		if (go) {
			carrot_ortho_body_frame << 100, 0, 0;
		}
		else if (carrot_ortho_body_frame(0) == 100) {
			carrot_ortho_body_frame << 5, 0, 0;
		}
	}

	void UpdateMotionLibraryRollPitch(double roll, double pitch) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
			motion_library_ptr->setRollPitch(roll, pitch);
		}
	}

	void PublishOrthoBodyTransform(double roll, double pitch) {
		static tf2_ros::TransformBroadcaster br;
  		geometry_msgs::TransformStamped transformStamped;
  
	    transformStamped.header.stamp = ros::Time::now();
	    transformStamped.header.frame_id = "body";
	    transformStamped.child_frame_id = "ortho_body";
	    transformStamped.transform.translation.x = 0.0;
	    transformStamped.transform.translation.y = 0.0;
	    transformStamped.transform.translation.z = 0.0;
	    tf2::Quaternion q_ortho;
	    q_ortho.setRPY(-roll, -pitch, 0);
	    transformStamped.transform.rotation.x = q_ortho.x();
	    transformStamped.transform.rotation.y = q_ortho.y();
	    transformStamped.transform.rotation.z = q_ortho.z();
	    transformStamped.transform.rotation.w = q_ortho.w();

	    br.sendTransform(transformStamped);
	}

	void UpdateCarrotOrthoBodyFrame() {
		geometry_msgs::TransformStamped tf;
	    try {

	      tf = tf_buffer_.lookupTransform("ortho_body", "world", 
	                                    ros::Time(0), ros::Duration(1.0/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("ID 2 %s", ex.what());
	      return;
	    }

	    geometry_msgs::PoseStamped pose_global_goal_world_frame = PoseFromVector3(carrot_world_frame, "world");
	    geometry_msgs::PoseStamped pose_global_goal_ortho_body_frame = PoseFromVector3(Vector3(0,0,0), "ortho_body");
	   
	    tf2::doTransform(pose_global_goal_world_frame, pose_global_goal_ortho_body_frame, tf);
	    carrot_ortho_body_frame = VectorFromPose(pose_global_goal_ortho_body_frame);
	}

	void UpdateAttitudeGeneratorRollPitch(double roll, double pitch) {
		attitude_generator.UpdateRollPitch(roll, pitch);
	}

	void UpdateLaserRDFFramesFromPose() {
		transformAccelerationsIntoLaserRDFFrames();
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
    	if (motion_library_ptr != nullptr) {
    		Vector3 initial_acceleration = motion_library_ptr->getInitialAcceleration();
			motion_library_ptr->setInitialAccelerationLASER(transformOrthoBodyIntoLaserFrame(initial_acceleration));
			motion_library_ptr->setInitialAccelerationRDF(transformOrthoBodyIntoRDFFrame(initial_acceleration));
		}
	}


	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		//ROS_INFO("GOT POSE");
		attitude_generator.setZ(pose.pose.position.z);
		
		tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		mutex.lock();
		UpdateMotionLibraryRollPitch(roll, pitch);
		UpdateAttitudeGeneratorRollPitch(roll, pitch);
		PublishOrthoBodyTransform(roll, pitch);
		UpdateCarrotOrthoBodyFrame();
		UpdateLaserRDFFramesFromPose();
		mutex.unlock();

		ComputeBestAccelerationMotion();

		mutex.lock();
		SetPose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw);
		mutex.unlock();

		PublishHealthStatus();

		Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
		Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
		NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
		NanoMapPose nm_pose(pos, quat, nm_time);

		DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
		if (depth_image_collision_ptr != nullptr) {
			depth_image_collision_ptr->nanomap.AddPose(nm_pose);
		}
	}

	void PublishHealthStatus() {

		fla_msgs::ProcessStatus msg;
		msg.id = 21; // 21 is motion_primitives status_id
		msg.pid = getpid();

		msg.status = fla_msgs::ProcessStatus::READY;
		msg.arg = 0;

		if (!got_camera_info && use_depth_image) {
			msg.status = fla_msgs::ProcessStatus::ALARM;
			msg.arg = 1;  
		}
		
		status_pub.publish(msg);

	}

	void SetPose(double x, double y, double z, double yaw) {
		pose_global_x = x;
		pose_global_y = y;
		pose_global_z = z;
		pose_global_yaw = yaw;
	}

	void transformAccelerationsIntoLaserRDFFrames() {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {

			std::vector<Motion>::iterator motion_iterator_begin = motion_library_ptr->GetMotionNonConstIteratorBegin();
	  		std::vector<Motion>::iterator motion_iterator_end = motion_library_ptr->GetMotionNonConstIteratorEnd();

	  		Vector3 acceleration_ortho_body;
			Vector3 acceleration_laser_frame;
			Vector3 acceleration_rdf_frame;


	  		for (auto motion = motion_iterator_begin; motion != motion_iterator_end; motion++) {
	  			acceleration_ortho_body = motion->getAcceleration();

	  			acceleration_laser_frame = transformOrthoBodyIntoLaserFrame(acceleration_ortho_body);
	  			motion->setAccelerationLASER(acceleration_laser_frame);

	  			acceleration_rdf_frame = transformOrthoBodyIntoRDFFrame(acceleration_ortho_body);
	  			motion->setAccelerationRDF(acceleration_rdf_frame);
	  		} 
	  	}
	}

	Vector3 transformOrthoBodyIntoLaserFrame(Vector3 const& ortho_body_vector) {
		geometry_msgs::TransformStamped tf;
    	try {
     		tf = tf_buffer_.lookupTransform("laser", "ortho_body", 
                                    ros::Time(0), ros::Duration(1/30.0));
   		} catch (tf2::TransformException &ex) {
     	 	ROS_ERROR("ID 3 %s", ex.what());
      	return Vector3(0,0,0);
    	}
    	geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_vector, "ortho_body");
    	geometry_msgs::PoseStamped pose_vector_laser_frame = PoseFromVector3(Vector3(0,0,0), "laser");
    	tf2::doTransform(pose_ortho_body_vector, pose_vector_laser_frame, tf);
    	return VectorFromPose(pose_vector_laser_frame);
	}

	Vector3 transformOrthoBodyIntoRDFFrame(Vector3 const& ortho_body_vector) {
		geometry_msgs::TransformStamped tf;
		if (!got_camera_info) {
			return Vector3(0,0,0);
		}
    	try {
     		tf = tf_buffer_.lookupTransform(depth_sensor_frame, "ortho_body", 
                                    ros::Time(0), ros::Duration(1/30.0));
   		} catch (tf2::TransformException &ex) {
     	 	ROS_ERROR("ID 4 %s", ex.what());
      	return Vector3(0,0,0);
    	}
    	geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_vector, "ortho_body");
    	geometry_msgs::PoseStamped pose_vector_rdf_frame = PoseFromVector3(Vector3(0,0,0), depth_sensor_frame);
    	tf2::doTransform(pose_ortho_body_vector, pose_vector_rdf_frame, tf);
    	return VectorFromPose(pose_vector_rdf_frame);
	}

	Matrix3 GetOrthoBodyToRDFRotationMatrix() {
		geometry_msgs::TransformStamped tf;
		if (!got_camera_info) {
			return Matrix3();
		}
    	try {
     		tf = tf_buffer_.lookupTransform(depth_sensor_frame, "ortho_body", 
                                    ros::Time(0), ros::Duration(1/30.0));
   		} catch (tf2::TransformException &ex) {
     	 	ROS_ERROR("ID 5 %s", ex.what());
      	return Matrix3();
    	}
    	Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
	    Matrix3 R = quat.toRotationMatrix();
	    return R;
	}

	Matrix3 GetBodyToRDFRotationMatrix() {
		geometry_msgs::TransformStamped tf;
		if (!got_camera_info) {
			std::cout << "hadn't got camera info yet" << std::endl;
			return Matrix3();
		}
    	try {
     		tf = tf_buffer_.lookupTransform(depth_sensor_frame, "body", 
                                    ros::Time(0), ros::Duration(1/30.0));
   		} catch (tf2::TransformException &ex) {
     	 	ROS_ERROR("ID 5 %s", ex.what());
      	return Matrix3();
    	}
    	Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
	    Matrix3 R = quat.toRotationMatrix();
	    return R;
	}

	Vector3 TransformWorldToOrthoBody(Vector3 const& world_frame) {
		geometry_msgs::TransformStamped tf;
	    try {
	      tf = tf_buffer_.lookupTransform("ortho_body", "world", 
	                                    ros::Time(0), ros::Duration(1/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("ID 6 %s", ex.what());
	      return Vector3(1,1,1);
	    }

	    Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
	    Matrix3 R = quat.toRotationMatrix();
	    return R*world_frame;
	}

	void UpdateMotionLibraryVelocity(Vector3 const& velocity_ortho_body_frame) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
			motion_library_ptr->setInitialVelocity(velocity_ortho_body_frame);
			motion_library_ptr->setInitialVelocityLASER(transformOrthoBodyIntoLaserFrame(velocity_ortho_body_frame));
			motion_library_ptr->setInitialVelocityRDF(transformOrthoBodyIntoRDFFrame(velocity_ortho_body_frame));
		}
	}

	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		//ROS_INFO("GOT VELOCITY");
		attitude_generator.setZvelocity(twist.twist.linear.z);
		Vector3 velocity_world_frame(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
		Vector3 velocity_ortho_body_frame = TransformWorldToOrthoBody(velocity_world_frame);
		velocity_ortho_body_frame(2) = 0.0;  // WARNING for 2D only
		mutex.lock();
		UpdateMotionLibraryVelocity(velocity_ortho_body_frame);
		speed = velocity_ortho_body_frame.norm();
		//UpdateTimeHorizon(speed);
		UpdateMaxAcceleration(speed);
		mutex.unlock();
	}

	void UpdateMaxAcceleration(double speed) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
			if (motion_library_ptr != nullptr) {
				motion_library_ptr->UpdateMaxAcceleration(speed);
			}
	}

	void UpdateTimeHorizon(double speed) { 
		if (speed < 10.0) {
			final_time = 1.0;
		}
		else { 
			final_time = 10.0 / speed;
		}
		if (final_time < 1.0) { final_time = 1.0;}
		motion_visualizer.UpdateTimeHorizon(final_time);
		motion_selector.UpdateTimeHorizon(final_time);
	}
	
	Vector3 TransformOrthoBodyToWorld(Vector3 const& ortho_body_frame) {
		geometry_msgs::TransformStamped tf;
	    try {
	      tf = tf_buffer_.lookupTransform("world", "ortho_body",
	                                    ros::Time(0), ros::Duration(1/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("ID 7 %s", ex.what());
	      return Vector3::Zero();
	    }

	    geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_frame, "ortho_body");
    	geometry_msgs::PoseStamped pose_vector_world_frame = PoseFromVector3(Vector3(0,0,0), "world");
    	tf2::doTransform(pose_ortho_body_vector, pose_vector_world_frame, tf);
    	return VectorFromPose(pose_vector_world_frame);
	}

	void ProjectOrthoBodyLaserPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr) {
		pcl::PointCloud<pcl::PointXYZ>::iterator point_cloud_iterator_begin = cloud_ptr->begin();
		pcl::PointCloud<pcl::PointXYZ>::iterator point_cloud_iterator_end = cloud_ptr->end();

		for (pcl::PointCloud<pcl::PointXYZ>::iterator point = point_cloud_iterator_begin; point != point_cloud_iterator_end; point++) {
			if (point->z > laser_z_below_project_up) {
				point->z = 0.0;
			}
		}
	}


	void OnScan(sensor_msgs::PointCloud2ConstPtr const& laser_point_cloud_msg) {
		//ROS_INFO("GOT SCAN");
		DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();

		if (depth_image_collision_ptr != nullptr) {

			//sensor_msgs::PointCloud2ConstPtr laser_point_cloud_msg_ptr(laser_point_cloud_msg);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr ortho_body_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		    TransformToOrthoBodyPointCloud("laser", laser_point_cloud_msg, ortho_body_cloud);

		    if (!use_3d_library) {
		     	ProjectOrthoBodyLaserPointCloud(ortho_body_cloud);
		    }

			depth_image_collision_ptr->UpdateLaserPointCloudPtr(ortho_body_cloud);
		}
	}

	void UpdateValueGrid(nav_msgs::OccupancyGrid value_grid_msg) {
		auto t1 = std::chrono::high_resolution_clock::now();

		ValueGridEvaluator* value_grid_evaluator_ptr = motion_selector.GetValueGridEvaluatorPtr();
		if (value_grid_evaluator_ptr != nullptr) {
			ValueGrid* value_grid_ptr = value_grid_evaluator_ptr->GetValueGridPtr();
			if (value_grid_ptr != nullptr) {

				value_grid_ptr->SetResolution(value_grid_msg.info.resolution);
				value_grid_ptr->SetSize(value_grid_msg.info.width, value_grid_msg.info.height);
				value_grid_ptr->SetOrigin(value_grid_msg.info.origin.position.x, value_grid_msg.info.origin.position.y);
				value_grid_ptr->SetValues(value_grid_msg.data);

				//motion_selector.PassInUpdatedValueGrid(&value_grid);
				auto t2 = std::chrono::high_resolution_clock::now();
				std::cout << "Whole value grid construction took "
		      		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
		      		<< " microseconds\n";

				std::cout << value_grid_ptr->GetValueOfPosition(carrot_world_frame) << " is value of goal" << std::endl;
				std::cout << value_grid_ptr->GetValueOfPosition(Vector3(0,0,0)) << " is value of world origin" << std::endl;
				std::cout << value_grid_ptr->GetValueOfPosition(Vector3(0,-2.0,0)) << " is value of 1.5 to my right" << std::endl;
			}
		}
	}

	void OnValueGrid(nav_msgs::OccupancyGrid value_grid_msg) {
		ROS_INFO("GOT VALUE GRID");
		UpdateValueGrid(value_grid_msg);
	}

	// Hack function to add motion up/down in order to help state estimator
	// dolphin_altitude(t) = A * cos(t * 2pi/T) + flight_altitude
	// A = amplitude
	// T = period
        double A_dolphin = 0.5;
        double T_dolphin = 3.0;
        double dolphin_acceleration_threshold = 2.0;
        double cooldown_duration = 3.0;

        double time_of_last_dolphin_query = 0;
        bool dolphin_initialized = false;
        double time_of_start_cooldown = 0;

        double dolphin_offset = 0.0;

        size_t cooldown_hit_counter = 0;
        size_t cooldown_hit_threshold = 20;
        bool in_cooldown = false;


    // John-style Dolphin

    // if aggressive action chosen
    // dolphin cooldown timer = 0

    //if timer > dolphin_cooldown_time 
    	//dolphin_offset = sin(t)
    //else
    	//dolphin_offset *= 0.95;
    	//reset dolphin phase 


	double DolphinStrokeDetermineAltitude(double speed) {
		if (!dolphin_initialized) {
			dolphin_initialized = true;
			time_of_start_dolphin_stroke = ros::Time::now().toSec();
			time_of_start_cooldown = ros::Time::now().toSec();
			return flight_altitude;
		}

		double time_now = ros::Time::now().toSec();

		if (desired_acceleration.norm() > dolphin_acceleration_threshold) {
			cooldown_hit_counter++;
			if (cooldown_hit_counter > cooldown_hit_threshold) {
				cooldown_hit_counter = 0;
				time_of_start_cooldown = time_now;
				in_cooldown = true;	
			}
		}
		else {
			if (cooldown_hit_counter <= 1) {
				cooldown_hit_counter = 0;
			}
			else {
				cooldown_hit_counter--;
			}
		}
		
		if ( (time_now - time_of_start_cooldown) > cooldown_duration) {
			if (in_cooldown) {
				in_cooldown = false;
				time_of_start_dolphin_stroke = time_now;
			}
			double time_since_start_dolphin = time_now - time_of_start_dolphin_stroke;
			dolphin_offset = A_dolphin * sin(time_since_start_dolphin * 2 * M_PI / T_dolphin);

		}
		else {
			if (dolphin_offset > 0) {dolphin_offset = dolphin_offset - 0.01;}
			else {dolphin_offset = dolphin_offset + 0.01;}
		}

		double ans = dolphin_offset + flight_altitude;

		geometry_msgs::PoseStamped pose;
		pose.pose.position.z = ans;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "world";
		attitude_setpoint_visualization_pub.publish(pose);

		return ans;
	}


	void OnLocalGoal(geometry_msgs::PoseStamped const& local_goal) {
		//ROS_INFO("GOT LOCAL GOAL");
		mutex.lock();
		double dolphin_altitude = DolphinStrokeDetermineAltitude(speed);
		carrot_world_frame << local_goal.pose.position.x, local_goal.pose.position.y, dolphin_altitude; 
		if (!use_3d_library) {
			attitude_generator.setZsetpoint(dolphin_altitude);
		}

		UpdateCarrotOrthoBodyFrame();
		mutex.unlock();

		visualization_msgs::Marker marker;
		marker.header.frame_id = "ortho_body";
		marker.header.stamp = ros::Time::now();
		marker.ns = "carrot_namespace";
		marker.id = 1;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		mutex.lock();
		marker.pose.position.x = carrot_ortho_body_frame(0);
		marker.pose.position.y = carrot_ortho_body_frame(1);
		marker.pose.position.z = carrot_ortho_body_frame(2);
		mutex.unlock();
		marker.scale.x = 0.5;
		marker.scale.y = 0.5;
		marker.scale.z = 0.5;
		marker.color.a = 0.5; // Don't forget to set the alpha!
		marker.color.r = 0.9;
		marker.color.g = 0.4;
		marker.color.b = 0.0;
		carrot_pub.publish( marker );
	}

	void TransformToOrthoBodyPointCloud(std::string const& source_frame, const sensor_msgs::PointCloud2ConstPtr msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out){
	  	sensor_msgs::PointCloud2 msg_out;

	  	geometry_msgs::TransformStamped tf;
    	try {
	     	tf = tf_buffer_.lookupTransform("ortho_body", source_frame,
	                                    ros::Time(0), ros::Duration(1/30.0));
	   		} catch (tf2::TransformException &ex) {
	     	 	ROS_ERROR("8 %s", ex.what());
      	return;
    	}

	  	Eigen::Quaternionf quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
	    Eigen::Matrix3f R = quat.toRotationMatrix();

	    Eigen::Vector4f T = Eigen::Vector4f(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z, 1.0); 

     	Eigen::Matrix4f transform_eigen; // Your Transformation Matrix
		transform_eigen.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
		transform_eigen.block<3,3>(0,0) = R;
		transform_eigen.col(3) = T;

	  	pcl_ros::transformPointCloud(transform_eigen, *msg, msg_out);

		pcl::PCLPointCloud2 cloud2;
		pcl_conversions::toPCL(msg_out, cloud2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(cloud2,*cloud);

		cloud_out = cloud;
	}

	void OnDepthImage(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
		// ROS_INFO("GOT POINT CLOUD");
		if (UseDepthImage()) {
			DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();

			if (depth_image_collision_ptr != nullptr) {

		    	Matrix3 R = GetOrthoBodyToRDFRotationMatrix();
		    	//std::cout << "ortho_body_to_rdf " << R << std::endl;

		    	Matrix3 R_set = GetBodyToRDFRotationMatrix();
				//std::cout << "R_set " << R_set << std::endl;
				depth_image_collision_ptr->nanomap.SetBodyToRdf(R_set);

		    	mutex.lock();
				depth_image_collision_ptr->UpdateRotationMatrix(R);
				depth_image_collision_ptr->UpdateBodyToRdf(R_set);
				if(use_depth_image) {

					pcl::PCLPointCloud2 cloud2_rdf;
					pcl_conversions::toPCL(*point_cloud_msg, cloud2_rdf);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rdf(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::fromPCLPointCloud2(cloud2_rdf,*cloud_rdf);

					NanoMapTime nm_time(point_cloud_msg->header.stamp.sec, point_cloud_msg->header.stamp.nsec);
					depth_image_collision_ptr->nanomap.AddPointCloud(cloud_rdf, nm_time, point_cloud_msg->header.seq);

					//pcl::PointCloud<pcl::PointXYZ>::Ptr ortho_body_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		    		//TransformToOrthoBodyPointCloud(depth_sensor_frame, point_cloud_msg, ortho_body_cloud);
					//depth_image_collision_ptr->UpdatePointCloudPtr(ortho_body_cloud);
				}
				mutex.unlock();
			}
			ReactToSampledPointCloud();
		}
	}

	void PublishAttitudeSetpoint(Vector3 const& roll_pitch_thrust) { 

		using namespace Eigen;

		mavros_msgs::AttitudeTarget setpoint_msg;
		setpoint_msg.header.stamp = ros::Time::now();
		setpoint_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE 
			| mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE
			| mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE
			;
		
		mutex.lock();

		// // Limit size of bearing errors
		double bearing_error_cap = 30;
		double actual_bearing_azimuth_degrees = -pose_global_yaw * 180.0/M_PI;
		double actual_bearing_error = bearing_azimuth_degrees - actual_bearing_azimuth_degrees;
		while(actual_bearing_error > 180) { 
			actual_bearing_error -= 360;
		}
		while(actual_bearing_error < -180) { 
			actual_bearing_error += 360;
		}
		if (actual_bearing_error > bearing_error_cap) {
			bearing_azimuth_degrees = actual_bearing_azimuth_degrees + bearing_error_cap;
		}
		if (actual_bearing_error < -bearing_error_cap) {
			bearing_azimuth_degrees = actual_bearing_azimuth_degrees - bearing_error_cap;
		}

		double bearing_error = bearing_azimuth_degrees - set_bearing_azimuth_degrees;

		while(bearing_error > 180) { 
			bearing_error -= 360;
		}
		while(bearing_error < -180) { 
			bearing_error += 360;
		}

		if (abs(bearing_error) < 1.0) {
			set_bearing_azimuth_degrees = bearing_azimuth_degrees;
		}

		else if (bearing_error < 0)  {
			set_bearing_azimuth_degrees -= 1.0;
		}
		else {
			set_bearing_azimuth_degrees += 1.0;
		}
		if (set_bearing_azimuth_degrees > 180.0) {
			set_bearing_azimuth_degrees -= 360.0;
		}
		if (set_bearing_azimuth_degrees < -180.0) {
			set_bearing_azimuth_degrees += 360.0;
		}

		Matrix3f m;
		m =AngleAxisf(-set_bearing_azimuth_degrees*M_PI/180.0, Vector3f::UnitZ())
		* AngleAxisf(roll_pitch_thrust(1), Vector3f::UnitY())
		* AngleAxisf(-roll_pitch_thrust(0), Vector3f::UnitX());

		mutex.unlock();

		Quaternionf q(m);

		setpoint_msg.orientation.w = q.w();
		setpoint_msg.orientation.x = q.x();
		setpoint_msg.orientation.y = q.y();
		setpoint_msg.orientation.z = q.z();

		setpoint_msg.thrust = roll_pitch_thrust(2);

		attitude_thrust_pub.publish(setpoint_msg);
	}


	ros::Subscriber camera_info_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Subscriber depth_image_sub;
	ros::Subscriber global_goal_sub;
	ros::Subscriber local_goal_sub;
	ros::Subscriber value_grid_sub;
	ros::Subscriber laser_scan_sub;
	ros::Subscriber max_speed_sub;

	ros::Publisher carrot_pub;
	ros::Publisher gaussian_pub;
	ros::Publisher attitude_thrust_pub;
	ros::Publisher attitude_setpoint_visualization_pub;
	ros::Publisher status_pub;

	std::string depth_sensor_frame = "depth_sensor";

	std::vector<ros::Publisher> action_paths_pubs;
	tf::TransformListener listener;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	tf2_ros::Buffer tf_buffer_;

	double start_time = 0.0;
	double final_time = 1.5;

	double bearing_azimuth_degrees = 0.0;
	double set_bearing_azimuth_degrees = 0.0;

	Eigen::Vector4d pose_x_y_z_yaw;

	std::mutex mutex;

	Vector3 carrot_world_frame;
	Vector3 carrot_ortho_body_frame = Vector3(0,0,0);

	size_t best_traj_index = 0;
	Vector3 desired_acceleration = Vector3(0,0,0);

	MotionSelector motion_selector;
	AttitudeGenerator attitude_generator;

	double pose_global_x = 0;
	double pose_global_y = 0;
	double pose_global_z = 0;
	double pose_global_yaw = 0;

	bool yaw_on = false;
	double soft_top_speed_max = 0.0;
	bool use_depth_image = true;
	bool use_3d_library = false;
	double flight_altitude = 1.0;

	bool executing_e_stop = false;
	double begin_e_stop_time = 0.0;
	double e_stop_time_needed = 0.0;
	double max_e_stop_pitch_degrees = 60.0;

	double laser_z_below_project_up = -0.5;
	double time_of_start_dolphin_stroke = 0.0;
	double speed = 0.0;

	enum StatusArg {
			NOMINAL = 0
	};

	ros::NodeHandle nh;

public:
	MotionVisualizer motion_visualizer;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing motion_selector_node" << std::endl;

	ros::init(argc, argv, "MotionSelectorNode");

	MotionSelectorNode motion_selector_node;

	ros::Rate spin_rate(100);

	auto t1 = std::chrono::high_resolution_clock::now();
	auto t2 = std::chrono::high_resolution_clock::now();

	size_t counter = 0;

	while (ros::ok()) {
		//t1 = std::chrono::high_resolution_clock::now();
		//motion_selector_node.ReactToSampledPointCloud();
		//t2 = std::chrono::high_resolution_clock::now();
		// std::cout << "ReactToSampledPointCloud took "
  //     		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
  //     		<< " microseconds\n";
		motion_selector_node.PublishCurrentAttitudeSetpoint();

		counter++;
		if (counter > 3) {
			counter = 0;
			motion_selector_node.drawAll();
			if (!motion_selector_node.UseDepthImage()) {
				motion_selector_node.ReactToSampledPointCloud();
			}
		}
      	
		ros::spinOnce();
		spin_rate.sleep();
	}
}
