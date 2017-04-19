#include "motion.h"

void Motion::setAccelerationMax(double const& acceleration_max) {
  this->a_max_horizontal = acceleration_max;
};

void Motion::setAcceleration(Vector3 const& acceleration) {
  this->acceleration = acceleration;
  this->unscaled_acceleration = acceleration;
};

void Motion::ScaleAcceleration(double scale_factor) {
  this->acceleration = unscaled_acceleration * scale_factor;
};

void Motion::setInitialAcceleration(Vector3 const& initial_acceleration_to_set) {
  initial_acceleration = initial_acceleration_to_set;
  jerk = (acceleration - initial_acceleration) / jerk_time;
  position_end_of_jerk_time = 0.1666*jerk*jerk_time*jerk_time*jerk_time + 0.5*initial_acceleration*jerk_time*jerk_time + initial_velocity*jerk_time;
  velocity_end_of_jerk_time = 0.5*jerk*jerk_time*jerk_time + initial_acceleration*jerk_time + initial_velocity;
};

void Motion::setInitialVelocity(Vector3 const& initial_velocity_to_set) {
  initial_velocity = initial_velocity_to_set;
};


Vector3 Motion::getAcceleration() const{
  return this->acceleration;
}

Vector3 Motion::getJerk() const {
  return this->jerk;
}

Vector3 Motion::getInitialVelocity() const {
  return initial_velocity;
};

Vector3 Motion::getVelocity(Scalar const& t) const {
  if (t < jerk_time) {
    return 0.5*jerk*t*t + initial_acceleration*t + initial_velocity; 
  }
  else {
    double t_left = t - jerk_time;
    return velocity_end_of_jerk_time + acceleration*t_left;
  }
};

Vector3 Motion::getPosition(Scalar const& t) const {
  if (t < jerk_time) {
    return 0.1666*jerk*t*t*t + 0.5*initial_acceleration*t*t + initial_velocity*t;
  }
  else {
    double t_left = t - jerk_time;
    return position_end_of_jerk_time + 0.5*acceleration*t_left*t_left + velocity_end_of_jerk_time*t_left;
  }
};

Vector3 Motion::getTerminalStopPosition(Scalar const& t) const {
  Vector3 position_end_of_motion = getPosition(t);
  Vector3 velocity_end_of_motion = getVelocity(t);

  double speed = velocity_end_of_motion.norm();
  
  Vector3 stopping_vector = -velocity_end_of_motion/speed;
  Vector3 max_stop_acceleration = a_max_horizontal*stopping_vector;
  Vector3 stopping_jerk = (max_stop_acceleration - acceleration) / jerk_time;
  Vector3 position_end_of_jerk_stop = 0.1666*stopping_jerk*jerk_time*jerk_time*jerk_time + 0.5*acceleration*jerk_time*jerk_time + velocity_end_of_motion*jerk_time + position_end_of_motion;
  Vector3 velocity_end_of_jerk_stop = 0.5*stopping_jerk*jerk_time*jerk_time + acceleration*jerk_time + velocity_end_of_motion;

  // check if stopped during jerk time
  if (velocity_end_of_motion.dot(velocity_end_of_jerk_stop) < 0) {
    return position_end_of_jerk_stop;
  }

  double realistic_stop_accel = a_max_horizontal*stopping_factor;
  double speed_after_jerk = velocity_end_of_jerk_stop.norm();
  double stop_t_after_jerk = (speed_after_jerk / realistic_stop_accel);
  //double extra_drift = speed_after_jerk*0.200;
  double stopping_distance_after_jerk =  0.5 * -realistic_stop_accel * stop_t_after_jerk*stop_t_after_jerk + speed_after_jerk*stop_t_after_jerk;

  return position_end_of_jerk_stop + stopping_distance_after_jerk*-stopping_vector;

}


