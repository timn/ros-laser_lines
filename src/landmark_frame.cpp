
/***************************************************************************
 *  laser_lines.cpp - Laser line detection and reference frame extraction
 *
 *  Created: Wed Oct 15 10:06:23 2014 (based on Fawkes laser-lines plugin)
 *  Copyright  2013-2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Library General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 *  Read the full text in the LICENSE file in the root directory.
 */

#include <ros/ros.h>

#include <laser_lines/LaserLines.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <angles/angles.h>

#include <yaml-cpp/yaml.h>
#ifndef HAVE_YAMLCPP_0_5
#  include <fstream>
#endif

#include <Eigen/Geometry>

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}


class Landmark {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string name;
  std::string frame;
  float bbox_x1;
  float bbox_y1;
  float bbox_x2;
  float bbox_y2;
  Eigen::Vector2f    intersection;
  Eigen::Vector3f    axis;
  Eigen::Vector3f    offset;
  Eigen::Quaternionf orientation;
};

static void
operator >> (const YAML::Node& n, Landmark &landmark)
{
  if (n.Type() != YAML::NodeType::Map) {
    throw std::runtime_error("Node is not a map!?");
  }

#ifdef HAVE_YAMLCPP_0_5
  landmark.name = n["name"].as<std::string>();
#else
  n["name"] >> landmark.name;
#endif

  if (n["bbox"].size() != 4) {
    throw std::runtime_error("Invalid position for node, "
			     "must be list of [x1,y1, x2, y2] coordinates");
  }
#ifdef HAVE_YAMLCPP_0_5
  landmark.bbox_x1 = n["bbox"][0].as<float>();
  landmark.bbox_y1 = n["bbox"][1].as<float>();
  landmark.bbox_x2 = n["bbox"][2].as<float>();
  landmark.bbox_y2 = n["bbox"][3].as<float>();
#else
  n["bbox"][0] >> landmark.bbox_x1;
  n["bbox"][1] >> landmark.bbox_y1;
  n["bbox"][2] >> landmark.bbox_x2;
  n["bbox"][3] >> landmark.bbox_y2;
#endif

  if (n["intersection"].size() != 2) {
    throw std::runtime_error("Invalid intersection, must be list of [b1, b2] values");
  }
#ifdef HAVE_YAMLCPP_0_5
  landmark.intersection[0] = n["intersection"][0].as<float>();
  landmark.intersection[1] = n["intersection"][1].as<float>();
#else
  n["intersection"][0] >> landmark.intersection[0];
  n["intersection"][1] >> landmark.intersection[1];
#endif

  if (n["axis"].size() != 3) {
    throw std::runtime_error("Invalid axis, must be list of [x,y,z]");
  }
#ifdef HAVE_YAMLCPP_0_5
  landmark.axis[0] = n["axis"][0].as<float>();
  landmark.axis[1] = n["axis"][1].as<float>();
  landmark.axis[2] = n["axis"][2].as<float>();
#else
  n["axis"][0] >> landmark.axis[0];
  n["axis"][1] >> landmark.axis[1];
  n["axis"][2] >> landmark.axis[2];
#endif
  landmark.axis.normalize();

  if (n["offset"].size() != 3) {
    throw std::runtime_error("Invalid offset, must be list of [x,y,z] offset");
  }
#ifdef HAVE_YAMLCPP_0_5
  landmark.offset[0] = n["offset"][0].as<float>();
  landmark.offset[1] = n["offset"][1].as<float>();
  landmark.offset[2] = n["offset"][2].as<float>();
#else
  n["offset"][0] >> landmark.offset[0];
  n["offset"][1] >> landmark.offset[1];
  n["offset"][2] >> landmark.offset[2];
#endif

  if (n["orientation"].size() != 3 && n["orientation"].size() != 4) {
    throw std::runtime_error("Invalid orientation, must be list of [r,p,y] or [x,y,z,w]");
  }
  if (n["orientation"].size() == 3) {
    float roll, pitch, yaw;
#ifdef HAVE_YAMLCPP_0_5
    roll  = n["orientation"][0].as<float>();
    pitch = n["orientation"][1].as<float>();
    yaw   = n["orientation"][2].as<float>();
#else
    n["orientation"][0] >> roll;
    n["orientation"][1] >> pitch;
    n["orientation"][2] >> yaw;
#endif

    landmark.orientation =
      Eigen::AngleAxisf(angles::from_degrees(roll), Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(angles::from_degrees(pitch), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(angles::from_degrees(yaw), Eigen::Vector3f::UnitZ());

  } else {
    float x, y, z, w;
#ifdef HAVE_YAMLCPP_0_5
    x = n["orientation"][0].as<float>();
    y = n["orientation"][1].as<float>();
    z = n["orientation"][2].as<float>();
    w = n["orientation"][3].as<float>();
#else
    n["orientation"][0] >> x;
    n["orientation"][1] >> y;
    n["orientation"][2] >> z;
    n["orientation"][3] >> w;
#endif

    landmark.orientation = Eigen::Quaternionf(w, x, y, z);
  }

  bool has_frame = true;
  try {
#ifdef HAVE_YAMLCPP_0_5
    has_frame = n["frame"].IsDefined();
#else
    has_frame = (n.FindValue("frame") != NULL);
#endif
  } catch (YAML::Exception &e) {
    has_frame = false;
  }

  if (has_frame) {
#ifdef HAVE_YAMLCPP_0_5
    landmark.frame = n["frame"].as<std::string>();
#else
    n["frame"] >> landmark.frame;
#endif
  }
}


class LandmarkFrame {
 public:

  LandmarkFrame()
    : last_id_num_(5)
  {
    ros::NodeHandle privnh("~");

    last_odom_pose_[0] = 0.;
    last_odom_pose_[1] = 0.;
    last_odom_pose_[2] = 0.;
    last_odom_time_ = ros::Time(0);
    transform_valid_ = false;

    privnh.param("default_frame", cfg_default_frame_, std::string("/landmark"));
    privnh.param("global_frame", cfg_global_frame_, std::string("/map"));
    privnh.param("odom_frame", cfg_odom_frame_, std::string("odom"));
    privnh.param("base_frame", cfg_base_frame_, std::string("base_link"));
    privnh.param("intersection_tolerance", cfg_intersection_tolerance_, 0.4);
    privnh.param("thresh_d", cfg_thresh_d_, 0.05);
    privnh.param("thresh_a", cfg_thresh_a_, 0.05);
    privnh.param("thresh_t", cfg_thresh_t_, 2.0);

    printf("default_frame %s\n", cfg_default_frame_.c_str());
    printf("global_frame %s\n", cfg_global_frame_.c_str());
    printf("odom_frame %s\n", cfg_odom_frame_.c_str());
    printf("base_frame %s\n", cfg_base_frame_.c_str());
    printf("intersection_tolerance %f\n", cfg_intersection_tolerance_);
    printf("thresh_d %f\n", cfg_thresh_d_);
    printf("thresh_a %f\n", cfg_thresh_a_);
    printf("thresh_t %f\n", cfg_thresh_t_);

    std::string filename;
    if (! privnh.getParam("file", filename)) {
      throw std::runtime_error("FAIL: you must pass a config file parameter");
    }
    read_config(filename);
    print_config();

    pub_vis_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, false);
    sub_lines_      = nh_.subscribe("lines", 1, &LandmarkFrame::process_lines, this);
  }

  void process_lines(const laser_lines::LaserLines::ConstPtr &lines)
  {
    // *** get active configuration

    //It would be nicer to use lines->header.stamp, but that frequently fails,
    // probably because /map is published to seldomly. Since we assume a still
    // standing robot we should be fine just using the latest time.
    tf::Stamped<tf::Pose> ident =
      tf::Stamped<tf::Pose>(tf::Transform(tf::Quaternion(0, 0, 0, 1),
					  tf::Vector3(0, 0, 0)),
			    ros::Time(0,0), lines->header.frame_id);
    tf::Stamped<tf::Pose> pose;
    try {
      tf_listener_.transformPose(cfg_global_frame_, ident, pose);
    } catch (tf::TransformException &e) {
      ROS_WARN("Failed to get robot pose with respect to sensor frame: %s", e.what());
      return;
    }

    Landmark lm;
    std::list<Landmark>::iterator l;
    for (l = landmarks_.begin(); l != landmarks_.end(); ++l) {

      float pose_x = pose.getOrigin().x();
      float pose_y = pose.getOrigin().y();

      if ( pose_x >= l->bbox_x1 && pose_x <= l->bbox_x2 &&
	   pose_y >= l->bbox_y1 && pose_y <= l->bbox_y2 )
      {
	lm = *l;
	break;
      }
    }
    if (l == landmarks_.end()) {
      ROS_WARN("No matching landmark specification found");
      return;
    }

    // *** check if we should actually update
    double odom_x = 0., odom_y = 0., odom_yaw = 0.;
    if(!get_odom_pose(odom_x, odom_y, odom_yaw, lines->header.stamp))
    {
      ROS_ERROR("Couldn't determine robot's pose associated with laser lines");
      return;
    }

    double delta_x = odom_x - last_odom_pose_[0];
    double delta_y = odom_y - last_odom_pose_[1];
    double delta_a = angle_diff(odom_yaw, last_odom_pose_[2]);
    double delta_t = (ros::Time::now() - last_odom_time_).toSec();

    bool odom_moved = fabs(delta_x) > cfg_thresh_d_ ||
                      fabs(delta_y) > cfg_thresh_d_ ||
                      fabs(delta_a) > cfg_thresh_a_;

    bool update = odom_moved || fabs(delta_t) < cfg_thresh_t_;

    if (! update) {
      if (transform_valid_) {
        ROS_INFO("Not moved, not updating frame but republishing old one");
        transform_.stamp_ = ros::Time::now();
        tf_bc_.sendTransform(transform_);
      } else {
        ROS_INFO("Not moved and no valid transform, yet. Simply skipping.");
      }
      return;
    }

    if (odom_moved) {
      last_odom_time_ = ros::Time::now();
    }
    last_odom_pose_[0] = odom_x;
    last_odom_pose_[1] = odom_y;
    last_odom_pose_[2] = odom_yaw;

    //ROS_INFO("Using landmark sector %s", lm.name.c_str());

    // find lines by checking the intersection
    Eigen::Vector2f best_intersection(0, 0), best_intersection_global(0,0);
    float best_intersection_distance = HUGE;
    Eigen::Vector3f best_pol_1(0,0,0), best_pol_2(0,0,0), best_ldir_1(0,0,0), best_ldir_2(0,0,0);

    // we make a strong assumption here:
    // the lines are all in the XY plane of the reference frame!

    for (size_t i = 0; i < lines->lines.size(); ++i) {
      const laser_lines::LaserLine &l1 = lines->lines[i];
      if (l1.point_on_line.z != 0) {
	ROS_WARN("Line (%f,%f,%f)->(%f,%f,%f) is not in the "
		 "XY-plane of the reference frame, ignoring",
		 l1.point_on_line.x, l1.point_on_line.y, l1.point_on_line.z,
		 l1.line_direction.x, l1.line_direction.y, l1.line_direction.z);
	continue;
      }
		 

      for (size_t j = 0; j < lines->lines.size(); ++j) {
	if (i == j) continue;

	const laser_lines::LaserLine &l2 = lines->lines[j];

	if (l2.point_on_line.z != 0)  continue;

	// calculate intersection
	Eigen::Vector2f point_on_line_1(l1.point_on_line.x, l1.point_on_line.y);
	Eigen::Vector2f line_direction_1(l1.line_direction.x, l1.line_direction.y);

	Eigen::Vector2f point_on_line_2(l2.point_on_line.x, l2.point_on_line.y);
	Eigen::Vector2f line_direction_2(l2.line_direction.x, l2.line_direction.y);

	Eigen::ParametrizedLine<float, 2> line_1(point_on_line_1, line_direction_1);
	Eigen::ParametrizedLine<float, 2> line_2(point_on_line_2, line_direction_2);

	float t = line_1.intersection(Eigen::Hyperplane<float, 2>(line_2));
#if EIGEN_VERSION_AT_LEAST(3,1,0)
	Eigen::Vector2f intersection = line_1.pointAt(t);
#else
	Eigen::Vector2f intersection = line_1.origin() + (line_1.direction() * t);
#endif

	// transform intersection to global frame to check distance

	tf::Stamped<tf::Point> ip =
	  tf::Stamped<tf::Point>(tf::Point(intersection[0], intersection[1], 0.),
				 ros::Time(0,0), lines->header.frame_id);
	tf::Stamped<tf::Point> ipg;
	try {
	  tf_listener_.transformPoint(cfg_global_frame_, ip, ipg);
	} catch (tf::TransformException &e) {
	  ROS_WARN("Failed to transform section to global frame: %s", e.what());
	  return;
	}

	Eigen::Vector2f intersection_global(ipg.x(), ipg.y());
	float distance = (lm.intersection - intersection_global).norm();

	//ROS_INFO("Intersection (%zu, %zu) @ (%f,%f), comparing to (%f,%f),  distance %f", i, j,
	//	 ipg.x(), ipg.y(), lm.intersection[0], lm.intersection[1], distance);

	if (distance < best_intersection_distance) {
	  best_intersection_distance = distance;
	  best_intersection          = intersection;
	  best_intersection_global   = intersection_global;
	  best_pol_1[0]              = point_on_line_1[0];
	  best_pol_1[1]              = point_on_line_1[1];
	  best_pol_2[0]              = point_on_line_2[0];
	  best_pol_2[1]              = point_on_line_2[1];
	  best_ldir_1[0]             = line_direction_1[0];
	  best_ldir_1[1]             = line_direction_1[1];
	  best_ldir_2[0]             = line_direction_2[0];
	  best_ldir_2[1]             = line_direction_2[1];
	}
	/*
	printf("Comparing (%f,%f) to (%f,%f), dist %f\n",
	       lm.intersection[0], lm.intersection[1],
	       intersection_global[0], intersection_global[1], distance);
	*/

	/*
	printf("(%f,%f,%f)->(%f,%f,%f) || (%f,%f,%f)->(%f,%f,%f) @ (%f,%f)/%f\n",
	       l1.point_on_line.x, l1.point_on_line.y, l1.point_on_line.z,
	       l1.line_direction.x, l1.line_direction.y, l1.line_direction.z,
	       l2.point_on_line.x, l2.point_on_line.y, l2.point_on_line.z,
	       l2.line_direction.x, l2.line_direction.y, l2.line_direction.z,
	       intersection[0], intersection[1], t);
	*/

      }
    }

    if (best_intersection_distance < cfg_intersection_tolerance_) {
      //ROS_INFO("Accepted intersection @ (%f,%f)",
      //       best_intersection[0], best_intersection[1]);

      // *** add offset to frame origin global
      
      tf::Stamped<tf::Vector3> line_dir_tf_1(tf::Vector3(best_ldir_1[0], best_ldir_1[1], best_ldir_1[2]),
					     ros::Time(0,0), lines->header.frame_id);
      tf::Stamped<tf::Vector3> line_dir_tf_2(tf::Vector3(best_ldir_1[0], best_ldir_1[1], best_ldir_1[2]),
					     ros::Time(0,0), lines->header.frame_id);

      tf::Stamped<tf::Vector3> line_dir_tf_1_g, line_dir_tf_2_g;
      try {
	tf_listener_.transformVector(cfg_global_frame_, line_dir_tf_1, line_dir_tf_1_g);
	tf_listener_.transformVector(cfg_global_frame_, line_dir_tf_2, line_dir_tf_2_g);
      } catch (tf::TransformException &e) {
	ROS_WARN("Failed to transform line directions to global frame: %s", e.what());
	return;
      }

      // 1. check which line is collinear with the landmark axis
    
      Eigen::Vector3f ldir_1_g(line_dir_tf_1_g.x(), line_dir_tf_2_g.y(), line_dir_tf_2_g.z());
      Eigen::Vector3f ldir_2_g(line_dir_tf_2_g.x(), line_dir_tf_2_g.y(), line_dir_tf_2_g.z());

      float angle_diff_1 = acosf(lm.axis.dot(ldir_1_g) / ldir_1_g.norm());
      float angle_diff_2 = acosf(lm.axis.dot(ldir_2_g) / ldir_2_g.norm());

      tf::Stamped<tf::Vector3> z_axis_tf_g(tf::Vector3(0,0,1), ros::Time(0,0), cfg_global_frame_);
      tf::Stamped<tf::Vector3> z_axis_tf;
      try {
	tf_listener_.transformVector(lines->header.frame_id, z_axis_tf_g, z_axis_tf);
      } catch (tf::TransformException &e) {
	ROS_WARN("Failed to transform Z axis to sensor frame: %s", e.what());
	return;
      }

      if (((angle_diff_1 > .25 * M_PI && fabs(M_PI - angle_diff_1) > .25 * M_PI)) ||
	  ((angle_diff_2 > .25 * M_PI && fabs(M_PI - angle_diff_1) > .25 * M_PI))) {
	ROS_WARN("Could not align an axis, ignoring");
	return;
      }
    

      Eigen::Vector3f x_dir((angle_diff_1 < angle_diff_2) ? ldir_1_g : ldir_2_g);
      Eigen::Vector3f z_dir(z_axis_tf.x(), z_axis_tf.y(), z_axis_tf.z());
      Eigen::Vector3f y_dir(z_dir.cross(x_dir));

      x_dir[2] = 0.f;

      x_dir.normalize();
      y_dir.normalize();
      z_dir.normalize();

      float angle_x_axes = acosf(lm.axis.dot(x_dir));
      if (angle_x_axes < .5 * M_PI) {
	// axes are not pointing in the same direction
	x_dir *= -1;
	y_dir *= -1;
      }

#if EIGEN_VERSION_AT_LEAST(3,1,0)
      Eigen::Quaternionf rotation =
	Eigen::Quaternionf::FromTwoVectors(lm.axis, x_dir) * lm.orientation.inverse();
#else
      Eigen::Quaternionf rotation;
      rotation.setFromTwoVectors(lm.axis, x_dir);
      rotation *= lm.orientation.inverse();
#endif

      Eigen::Vector3f bisg3(best_intersection_global[0], best_intersection_global[1], 0.);
      Eigen::Vector3f frame_origin(bisg3 + x_dir * lm.offset[0] + y_dir * lm.offset[1] + z_dir * lm.offset[2]);


      /*
      printf("xdir: (%f,%f,%f)   ydir (%f,%f,%f)   zdir (%f,%f,%f)  fog(%f,%f,%f)\n",
	     x_dir[0], x_dir[1], x_dir[2],
	     y_dir[0], y_dir[1], y_dir[2],
	     z_dir[0], z_dir[1], z_dir[2],
	     frame_origin[0], frame_origin[1], frame_origin[2]);


      Eigen::Vector3f fog(best_intersection_global[0], best_intersection_global[1], 0.f);
      fog += lm.offset;
      */

      // transform back to sensor frame
      tf::Stamped<tf::Pose> fpg =
	tf::Stamped<tf::Pose>(tf::Transform(tf::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()),
					    tf::Vector3(frame_origin[0], frame_origin[1], frame_origin[2])),
			      ros::Time(0,0), cfg_global_frame_);
      tf::Stamped<tf::Pose> fpgt;
      try {
	tf_listener_.transformPose(lines->header.frame_id, fpg, fpgt);
      } catch (tf::TransformException &e) {
	ROS_WARN("Failed to transform offset point to sensor frame: %s", e.what());
	return;
      }

      // publish transform
      transform_ = tf::StampedTransform(fpgt, ros::Time::now(), lines->header.frame_id, lm.frame);
      transform_valid_ = true;
      tf_bc_.sendTransform(transform_);

      // publish visualization
      visualization_msgs::MarkerArray m;
      unsigned int id_num = 0;

      visualization_msgs::Marker sphere;
      sphere.header.frame_id = lines->header.frame_id;
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "landmark_frame";
      sphere.id = id_num++;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = best_intersection[0];
      sphere.pose.position.y = best_intersection[1];
      sphere.pose.position.z = 0.;
      sphere.pose.orientation.w = 1.;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08;
      sphere.color.r = 1.f;
      sphere.color.g = 0.f;
      sphere.color.b = 0.f;
      sphere.color.a = 1.f;
      m.markers.push_back(sphere);

      for (size_t i = id_num; i < last_id_num_; ++i) {
	visualization_msgs::Marker delop;
	delop.header.frame_id = lines->header.frame_id;
	delop.header.stamp = ros::Time::now();
	delop.ns = "landmark_frame";
	delop.id = i;
	delop.action = visualization_msgs::Marker::DELETE;
	m.markers.push_back(delop);
      }
      pub_vis_marker_.publish(m);
      last_id_num_ = id_num;
    } else {
      ROS_WARN("Best intersection not within tolerance: %f > %f",
	       best_intersection_distance, cfg_intersection_tolerance_);
    }

  }


  bool
  get_odom_pose(double& x, double& y, double& yaw, const ros::Time& t)
  {
    tf::Stamped<tf::Pose> odom_pose;
    // Get the robot's pose
    tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
					      tf::Vector3(0,0,0)), t, cfg_base_frame_);
    if (! tf_listener_.waitForTransform(cfg_odom_frame_, cfg_base_frame_, t, ros::Duration(1.0))) {
      ROS_WARN("Waiting for transform timed out, skipping loop");
      return false;
    }
    try {
      tf_listener_.transformPose(cfg_odom_frame_, ident, odom_pose);
    } catch(tf::TransformException e) {
      ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      return false;
    }
    x = odom_pose.getOrigin().x();
    y = odom_pose.getOrigin().y();
    double pitch,roll;
    odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

    return true;
  }

  void read_config(const std::string &filename)
  {
    YAML::Node doc;
#ifdef HAVE_YAMLCPP_0_5
    if (! (doc = YAML::LoadFile(filename))) {
#else
    std::ifstream fin(filename.c_str());
    YAML::Parser parser(fin);
    if (! parser.GetNextDocument(doc)) {
#endif
      throw std::runtime_error("Failed to read YAML file");
    }

    const YAML::Node &nodes = doc["landmarks"];

#ifdef HAVE_YAMLCPP_0_5
    for (YAML::const_iterator n = nodes.begin(); n != nodes.end(); ++n) {
#else
    for (YAML::Iterator n = nodes.begin(); n != nodes.end(); ++n) {
#endif
      Landmark landmark;
      *n >> landmark;
      if (landmark.frame.empty())  landmark.frame = cfg_default_frame_;
      landmarks_.push_back(landmark);
    }
  }

  void print_config()
  {
    std::list<Landmark>::iterator l;
    for (l = landmarks_.begin(); l != landmarks_.end(); ++l) {
      Landmark &lm = *l;

      printf("Landmark %s: (%f,%f)--(%f,%f), I (%f,%f) O (%f,%f,%f)\n",
	     lm.name.c_str(), lm.bbox_x1, lm.bbox_y1, lm.bbox_x2, lm.bbox_y2,
	     lm.intersection[0], lm.intersection[1], lm.offset[0], lm.offset[1], lm.offset[2]);
    }
  }

 private:
  ros::NodeHandle nh_;

  std::string cfg_default_frame_;
  std::string cfg_global_frame_;
  std::string cfg_odom_frame_;
  std::string cfg_base_frame_;
  double      cfg_intersection_tolerance_;
  double      cfg_thresh_d_;
  double      cfg_thresh_a_;
  double      cfg_thresh_t_;

  ros::Subscriber          sub_lines_;
  ros::Publisher           pub_vis_marker_;
  tf::TransformBroadcaster tf_bc_;
  tf::TransformListener    tf_listener_;
  
  std::list<Landmark>  landmarks_;
  tf::StampedTransform transform_;
  bool                 transform_valid_;

  size_t last_id_num_;
  double last_odom_pose_[3];
  ros::Time last_odom_time_;
};




/** landmark_frame node application.
 * @param argc argument count
 * @param argv array of arguments
 */
int
main(int argc, char **argv)
{
  ros::init(argc, argv, "landmark_frame");

  LandmarkFrame landmark_frames;

  ros::spin();
  return 0;
}
