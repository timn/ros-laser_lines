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

// messages
#include <laser_lines/LaserLines.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <laser_geometry/laser_geometry.h>

//#define PCL16

#ifdef PCL16
#include <pcl16/point_cloud.h>
#include <pcl16/point_types.h>
#include <pcl16/segmentation/sac_segmentation.h>
#include <pcl16/segmentation/extract_clusters.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/filters/passthrough.h>
#include <pcl16/filters/project_inliers.h>
#include <pcl16/search/kdtree.h>
#include <pcl16/common/centroid.h>
#include <pcl16/common/transforms.h>
#include <pcl16/common/distances.h>

namespace pcl = pcl16;

#else

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>

#endif

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)

#include <pcl_conversions/pcl_conversions.h>

#endif

// Set this if you want to force the use of the old PCL code path.
// This is useful to test on newer hosts that the code for older ones
// should in principal work.
//#define FORCE_OLD_PCL

#include <Eigen/StdVector>

#define MAX_LINES 12

static const uint8_t line_colors[MAX_LINES][3] =
        {{255, 0,   0},
         {176, 0,   30},
         {255, 90,  0},
         {137, 82,  39},
         {56,  23,  90},
         {99,  0,   30},
         {0,   255, 0},
         {0,   0,   255},
         {255, 255, 0},
         {255, 0,   255},
         {0,   255, 255},
         {27,  117, 196}};

class LaserLines {

    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    typedef pcl::PointXYZRGB ColorPointType;
    typedef pcl::PointCloud<ColorPointType> ColorCloud;
    typedef ColorCloud::Ptr ColorCloudPtr;
    typedef ColorCloud::ConstPtr ColorCloudConstPtr;

    class LineInfo {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        unsigned int index;
        int visibility_history;
        float bearing;

        Eigen::Vector3f point_on_line;
        Eigen::Vector3f line_direction;

        Eigen::Vector3f base_point;

        CloudPtr cloud;

        Eigen::Vector3f point_1, point_2; // Extreme points of the line
        float length;

        bool operator<(const LineInfo &other) const { return bearing < other.bearing; }
    };

public:

    LaserLines()
            : last_id_num_(0) {
      ros::NodeHandle privnh("~");

      privnh.param("line_segmentation_max_iterations", cfg_segm_max_iterations_, 250);
      privnh.param("line_segmentation_distance_threshold", cfg_segm_distance_threshold_, 0.05);
      privnh.param("line_segmentation_sample_max_dist", cfg_segm_sample_max_dist_, 0.15);
      privnh.param("line_segmentation_min_inliers", cfg_segm_min_inliers_, 30);
      privnh.param("line_min_length", cfg_min_length_, 0.8);
      privnh.param("line_cluster_tolerance", cfg_line_cluster_tolerance_, 0.4);
      privnh.param("line_cluster_quota", cfg_line_cluster_quota_, 0.5);
      privnh.param("line_min_distance", cfg_line_min_distance_, 0.1);
      privnh.param("switch_tolerance", cfg_switch_tolerance_, 0.3);

      printf("line_segmentation_max_iterations %u\n", cfg_segm_max_iterations_);
      printf("line_segmentation_distance_threshold %f\n", cfg_segm_distance_threshold_);
      printf("line_segmentation_sample_max_dist %f\n", cfg_segm_sample_max_dist_);
      printf("line_segmentation_min_inliers %i\n", cfg_segm_min_inliers_);
      printf("line_min_length %f\n", cfg_min_length_);
      printf("line_min_distance %f\n", cfg_line_min_distance_);
      printf("line_cluster_tolerance %f\n", cfg_line_cluster_tolerance_);
      printf("line_cluster_quota %f\n", cfg_line_cluster_quota_);
      printf("switch_tolerance %f\n", cfg_switch_tolerance_);

      // these are actually unsigned, but since NodeHandle::param does
      // not support those, assert this...
      assert(cfg_segm_min_inliers_ > 0);
      assert(cfg_segm_max_iterations_ > 0);

      sub_scan_ = nh_.subscribe("scan", 1, &LaserLines::process_scan, this);
      pub_lines_ = nh_.advertise<laser_lines::LaserLines>("lines", 1, true);
      pub_vis_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, false);
      pub_vis_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("lines_cloud", 1, false);
    }


    void process_scan(const sensor_msgs::LaserScan::ConstPtr &scan) {
      std::vector<LineInfo> linfos;

      sensor_msgs::PointCloud2 cloud;
      laser_proj_.projectLaser(*scan, cloud);

      if (cloud.width * cloud.height <= 10) {
        // this can happen if run at startup. Since thread runs continuous
        // and not synchronized with main loop, but point cloud acquisition thread is
        // synchronized, we might start before any data has been read
        ROS_WARN("Empty voxelized point cloud, omitting loop");
        //TimeWait::wait(50000);
        publish(linfos, scan->header.frame_id, scan->header.stamp);
        return;
      }

      CloudPtr input(new Cloud());
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
      moveFromROSMsg(cloud, *input);
#else
      pcl::fromROSMsg(cloud, *input);
#endif

      CloudPtr in_cloud(new Cloud());

      {
        // Erase non-finite points
        pcl::PassThrough<PointType> passthrough;
        passthrough.setInputCloud(input);
        passthrough.filter(*in_cloud);
      }

      pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

      while (in_cloud->points.size() > (unsigned int) cfg_segm_min_inliers_) {
        // segment the largest linear component from the remaining cloud
        //logger->log_info(name(), "[l %u] %zu points left",
        //		     loop_count_, in_cloud->points.size());

        pcl::SACSegmentation<PointType> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(cfg_segm_max_iterations_);
        seg.setDistanceThreshold(cfg_segm_distance_threshold_);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0) && !defined(FORCE_OLD_PCL)
        pcl::search::KdTree<PointType>::Ptr
                search(new pcl::search::KdTree<PointType>);
        search->setInputCloud(in_cloud);
        seg.setSamplesMaxDist(cfg_segm_sample_max_dist_, search);
#endif
        seg.setInputCloud(in_cloud);
        seg.segment(*inliers, *coeff);
        if (inliers->indices.size() == 0) {
          // no line found
          printf("no line found");
          break;
        }

        // check for a minimum number of expected inliers
        if ((double) inliers->indices.size() < cfg_segm_min_inliers_) {
//          logger->log_warn(name(), "[L %u] no more lines (%zu inliers, required %u)",
//          	       loop_count_, inliers->indices.size(), cfg_segm_min_inliers_);
          break;
        }

        //logger->log_info(name(), "[L %u] Found line with %zu inliers",
        //		     loop_count_, inliers->indices.size());


        // Cluster within the line to make sure it is a contiguous line
        // the line search can output a line which combines lines at separate
        // ends of the field of view...

#if PCL_VERSION_COMPARE(<, 1, 7, 0) || defined(FORCE_OLD_PCL)
        CloudPtr line_cloud(new Cloud());
        {
          pcl::ExtractIndices<PointType> extract;
          extract.setInputCloud(in_cloud);
          extract.setIndices(inliers);
          extract.setNegative(false);
          extract.filter(*line_cloud);
        }
#endif

        std::vector<pcl::PointIndices> line_cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> line_ec;
        pcl::search::KdTree<PointType>::Ptr
                kdtree_line_cluster(new pcl::search::KdTree<PointType>());
        line_ec.setClusterTolerance(cfg_line_cluster_tolerance_);
        size_t min_size = (size_t) floorf(cfg_line_cluster_quota_ * inliers->indices.size());
        line_ec.setMinClusterSize(min_size);
        line_ec.setMaxClusterSize(inliers->indices.size());
#if PCL_VERSION_COMPARE(>=, 1, 7, 0) && !defined(FORCE_OLD_PCL)
        line_ec.setInputCloud(in_cloud);
        line_ec.setIndices(inliers);
        pcl::search::KdTree<PointType>::IndicesConstPtr
                search_indices(new std::vector<int>(inliers->indices));
        kdtree_line_cluster->setInputCloud(in_cloud, search_indices);
#else
        line_ec.setInputCloud(line_cloud);
        kdtree_line_cluster->setInputCloud(line_cloud);
#endif
        line_ec.setSearchMethod(kdtree_line_cluster);
        line_ec.extract(line_cluster_indices);

        pcl::PointIndices::Ptr line_cluster_index;
        if (!line_cluster_indices.empty()) {
          line_cluster_index = pcl::PointIndices::Ptr(new pcl::PointIndices(line_cluster_indices[0]));
        }


        // re-calculate coefficients based on line cluster only
        if (line_cluster_index) {
          pcl::SACSegmentation<PointType> segc;
          segc.setOptimizeCoefficients(true);
          segc.setModelType(pcl::SACMODEL_LINE);
          segc.setMethodType(pcl::SAC_RANSAC);
          segc.setMaxIterations(cfg_segm_max_iterations_);
          segc.setDistanceThreshold(cfg_segm_distance_threshold_);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0) && !defined(FORCE_OLD_PCL)
          segc.setInputCloud(in_cloud);
#else
          segc.setInputCloud(line_cloud);
#endif
          segc.setIndices(line_cluster_index);
          pcl::PointIndices::Ptr tmp_index(new pcl::PointIndices());
          segc.segment(*tmp_index, *coeff);
          *line_cluster_index = *tmp_index;
        }

        // Remove the linear or clustered inliers, extract the rest
        CloudPtr cloud_f(new Cloud());
        CloudPtr cloud_line(new Cloud());
        pcl::ExtractIndices<PointType> extract;
#if PCL_VERSION_COMPARE(>=, 1, 7, 0) && !defined(FORCE_OLD_PCL)
        extract.setInputCloud(in_cloud);
#else
        extract.setInputCloud(line_cluster_index ? line_cloud : in_cloud);
#endif
        extract.setIndices(line_cluster_index ? line_cluster_index : inliers);
        extract.setNegative(false);
        extract.filter(*cloud_line);

#if PCL_VERSION_COMPARE(<, 1, 7, 0) || defined(FORCE_OLD_PCL)
        // old PCL: always remove all that were considered inliers
        // as we could not filter the indices properly (PCL 1.5 seems to be broken)
        extract.setInputCloud(in_cloud);
        extract.setIndices(inliers);
#endif

        extract.setNegative(true);
        extract.filter(*cloud_f);
        *in_cloud = *cloud_f;

        if (!line_cluster_index) continue;

        LineInfo info;
        info.index = linfos.size();

//        printf("info.index %lu\n", linfos.size());
        info.cloud.reset(new Cloud());

        // Check if this line has the requested minimum length
        calc_line_length(cloud_line, coeff, info.length, info.point_1, info.point_2);
        if (info.length < cfg_min_length_) {
          continue;
        }

        info.point_on_line[0] = coeff->values[0];
        info.point_on_line[1] = coeff->values[1];
        info.point_on_line[2] = coeff->values[2];
        info.line_direction[0] = coeff->values[3];
        info.line_direction[1] = coeff->values[4];
        info.line_direction[2] = coeff->values[5];

        Eigen::Vector3f ld_unit = info.line_direction / info.line_direction.norm();
        Eigen::Vector3f pol_invert = Eigen::Vector3f(0, 0, 0) - info.point_on_line;
        Eigen::Vector3f P = info.point_on_line + pol_invert.dot(ld_unit) * ld_unit;
        Eigen::Vector3f x_axis(1, 0, 0);
        info.bearing = acosf(x_axis.dot(P) / P.norm());
        // we also want to encode the direction of the angle
        if (coeff->values[1] < 0) info.bearing *= -1.;

        info.base_point = P;

        if (info.base_point.norm() < cfg_line_min_distance_) {
          //logger->log_warn(name(), "[L %u] line too short (%f, required %f)",
          //	       loop_count_, info.base_point.norm(), 0.25);
          continue;
        }

        coeff->values[0] = P[0];
        coeff->values[1] = P[1];
        coeff->values[2] = P[2];

        // Project the model inliers
        pcl::ProjectInliers<PointType> proj;
        proj.setModelType(pcl::SACMODEL_LINE);
        proj.setInputCloud(cloud_line);
        proj.setModelCoefficients(coeff);
        proj.filter(*info.cloud);

        linfos.push_back(info);
      }

      // sort lines by bearing to stabilize IDs (requires LineInfo::operator<)
      std::sort(linfos.begin(), linfos.end());

      publish(linfos, scan->header.frame_id, scan->header.stamp);
      publish_visualization(linfos, scan->header.frame_id);
      last_lines_ = linfos;
    }


    void calc_line_length(CloudPtr cloud_line, pcl::ModelCoefficients::Ptr coeff, float &length, Eigen::Vector3f &p1, Eigen::Vector3f &p2) {
      if (cloud_line->points.size() < 2) return ;

      //CloudPtr cloud_line(new Cloud());
      CloudPtr cloud_line_proj(new Cloud());

      // Project the model inliers
      pcl::ProjectInliers<PointType> proj;
      proj.setModelType(pcl::SACMODEL_LINE);
      proj.setInputCloud(cloud_line);
      proj.setModelCoefficients(coeff);
      proj.filter(*cloud_line_proj);

      Eigen::Vector3f point_on_line, line_dir;
      point_on_line[0] = cloud_line_proj->points[0].x;
      point_on_line[1] = cloud_line_proj->points[0].y;
      point_on_line[2] = cloud_line_proj->points[0].z;
      line_dir[0] = coeff->values[3];
      line_dir[1] = coeff->values[4];
      line_dir[2] = coeff->values[5];
      line_dir.normalize();

      ssize_t idx_1 = 0, idx_2 = 0;
      float max_dist_1 = 0.f, max_dist_2 = 0.f;

      for (size_t i = 1; i < cloud_line_proj->points.size(); ++i) {
        const PointType &pt = cloud_line_proj->points[i];
        Eigen::Vector3f ptv(pt.x, pt.y, pt.z);
        Eigen::Vector3f diff(ptv - point_on_line);
        float dist = diff.norm();
        float dir = line_dir.dot(diff);
        if (dir >= 0) {
          if (dist > max_dist_1) {
            max_dist_1 = dist;
            idx_1 = i;
          }
        }
        if (dir <= 0) {
          if (dist > max_dist_2) {
            max_dist_2 = dist;
            idx_2 = i;
          }
        }
      }

      if (idx_1 >= 0 && idx_2 >= 0) {
        const PointType &pt_1 = cloud_line_proj->points[idx_1];
        const PointType &pt_2 = cloud_line_proj->points[idx_2];

        Eigen::Vector3f ptv_1(pt_1.x, pt_1.y, pt_1.z);
        Eigen::Vector3f ptv_2(pt_2.x, pt_2.y, pt_2.z);

        length =  (ptv_1 - ptv_2).norm();

        // Extreme points
        p1[0] = pt_1.x;
        p1[1] = pt_1.y;
        p1[2] = pt_1.z;

        p2[0] = pt_2.x;
        p2[1] = pt_2.y;
        p2[2] = pt_2.z;

      } else {
        length = 0.f;
      }      
    }

    void publish(std::vector<LineInfo> &lines, const std::string &frame_id, const ros::Time &time) {
      laser_lines::LaserLines lines_msg;
      lines_msg.header.stamp = time;
      lines_msg.header.frame_id = frame_id;
      for (size_t i = 0; i < lines.size(); ++i) {
        laser_lines::LaserLine line_msg;
        LineInfo &li = lines[i];

        if (i < last_lines_.size()) {
          const LineInfo &lli = last_lines_[i];
          float diff = (lli.point_on_line - li.base_point).norm();

          if (lli.visibility_history >= 0 && (diff <= cfg_switch_tolerance_)) {
            li.visibility_history = lli.visibility_history + 1;
          } else {
            li.visibility_history = 1;
          }
        } else {
          li.visibility_history = 1;
        }

        line_msg.visibility_history = li.visibility_history;
        line_msg.bearing = li.bearing;

        line_msg.point_on_line.x = li.point_on_line[0];
        line_msg.point_on_line.y = li.point_on_line[1];
        line_msg.point_on_line.z = li.point_on_line[2];

        line_msg.line_direction.x = li.line_direction[0];
        line_msg.line_direction.y = li.line_direction[1];
        line_msg.line_direction.z = li.line_direction[2];

        line_msg.point_1.x = li.point_1[0];
        line_msg.point_1.y = li.point_1[1];
        line_msg.point_1.z = li.point_1[2];
        
        line_msg.point_2.x = li.point_2[0];
        line_msg.point_2.y = li.point_2[1];
        line_msg.point_2.z = li.point_2[2];

        line_msg.length = li.length;

        lines_msg.lines.push_back(line_msg);
      }
      pub_lines_.publish(lines_msg);
    }


    void publish_visualization(std::vector<LineInfo> &linfos, const std::string &frame_id) {
      // publish projected colored point cloud
      //
      size_t num_points = 0;
      for (size_t i = 0; i < linfos.size(); ++i) {
        num_points += linfos[i].cloud->points.size();
      }

      ColorCloudPtr lines_pcl(new ColorCloud());

      lines_pcl->points.resize(num_points);
      lines_pcl->height = 1;
      lines_pcl->width = num_points;

      size_t oi = 0;
      for (size_t i = 0; i < linfos.size(); ++i) {
        LineInfo &li = linfos[i];

        for (size_t p = 0; p < li.cloud->points.size(); ++p) {
          ColorPointType &out_point = lines_pcl->points[oi++];
          PointType &in_point = li.cloud->points[p];
          out_point.x = in_point.x;
          out_point.y = in_point.y;
          out_point.z = in_point.z;

          if (i < MAX_LINES) {
            out_point.r = line_colors[i][0];
            out_point.g = line_colors[i][1];
            out_point.b = line_colors[i][2];
          } else {
            out_point.r = out_point.g = out_point.b = 1.0;
          }
        }
      }

      sensor_msgs::PointCloud2 out_cloud;
      pcl::toROSMsg(*lines_pcl, out_cloud);
      out_cloud.header.stamp = ros::Time::now();
      out_cloud.header.frame_id = frame_id;

      pub_vis_cloud_.publish(out_cloud);


      // publish markers
      //
      visualization_msgs::MarkerArray m;
      unsigned int idnum = 0;

      for (size_t i = 0; i < linfos.size(); ++i) {
        const LineInfo &info = linfos[i];
        visualization_msgs::Marker dirvec;
        dirvec.header.frame_id = frame_id;
        dirvec.header.stamp = ros::Time::now();
        dirvec.ns = "laser_lines";
        dirvec.id = idnum++;
        dirvec.type = visualization_msgs::Marker::ARROW;
        dirvec.action = visualization_msgs::Marker::ADD;
        dirvec.points.resize(2);
        dirvec.points[0].x = info.base_point[0];
        dirvec.points[0].y = info.base_point[1];
        dirvec.points[0].z = info.base_point[2];
        dirvec.points[1].x = info.base_point[0] + info.line_direction[0];
        dirvec.points[1].y = info.base_point[1] + info.line_direction[1];
        dirvec.points[1].z = info.base_point[2] + info.line_direction[2];
        dirvec.scale.x = 0.02;
        dirvec.scale.y = 0.04;
        dirvec.color.r = 0.0;
        dirvec.color.g = 1.0;
        dirvec.color.b = 0.f;
        dirvec.color.a = 1.0;
        dirvec.lifetime = ros::Duration(2, 0);
        m.markers.push_back(dirvec);

        visualization_msgs::Marker testvec;
        testvec.header.frame_id = frame_id;
        testvec.header.stamp = ros::Time::now();
        testvec.ns = "laser_lines";
        testvec.id = idnum++;
        testvec.type = visualization_msgs::Marker::ARROW;
        testvec.action = visualization_msgs::Marker::ADD;
        testvec.points.resize(2);
        testvec.points[0].x = 0; //info.point_on_line[0];
        testvec.points[0].y = 0; //info.point_on_line[1];
        testvec.points[0].z = 0; //info.point_on_line[2];
        testvec.points[1].x = info.base_point[0];
        testvec.points[1].y = info.base_point[1];
        testvec.points[1].z = info.base_point[2];
        testvec.scale.x = 0.02;
        testvec.scale.y = 0.04;
        testvec.color.r = line_colors[i][0] / 255.;
        testvec.color.g = line_colors[i][1] / 255.;
        testvec.color.b = line_colors[i][2] / 255.;
        testvec.color.a = 1.0;
        testvec.lifetime = ros::Duration(2, 0);
        m.markers.push_back(testvec);

        char *tmp;
        if (asprintf(&tmp, "L_%zu", i + 1) != -1) {
          // Copy to get memory freed on exception
          std::string id = tmp;
          free(tmp);

          visualization_msgs::Marker text;
          text.header.frame_id = frame_id;
          text.header.stamp = ros::Time::now();
          text.ns = "laser_lines";
          text.id = idnum++;
          text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          text.action = visualization_msgs::Marker::ADD;
          text.pose.position.x = info.base_point[0];
          text.pose.position.y = info.base_point[1];
          text.pose.position.z = info.base_point[2] + .15;
          text.pose.orientation.w = 1.;
          text.scale.z = 0.15;
          text.color.r = text.color.g = text.color.b = 1.0f;
          text.color.a = 1.0;
          text.lifetime = ros::Duration(2, 0);
          text.text = id;
          m.markers.push_back(text);
        }
      }

      for (size_t i = idnum; i < last_id_num_; ++i) {
        visualization_msgs::Marker delop;
        delop.header.frame_id = frame_id;
        delop.header.stamp = ros::Time::now();
        delop.ns = "laser_lines";
        delop.id = i;
        delop.action = visualization_msgs::Marker::DELETE;
        m.markers.push_back(delop);
      }
      last_id_num_ = idnum;

      pub_vis_marker_.publish(m);
    }

private:
    ros::NodeHandle nh_;

    ros::Publisher pub_lines_;
    ros::Publisher pub_vis_marker_;
    ros::Publisher pub_vis_cloud_;
    ros::Subscriber sub_scan_;

    int cfg_segm_max_iterations_;
    double cfg_segm_distance_threshold_;
    double cfg_segm_sample_max_dist_;
    double cfg_min_length_;
    double cfg_line_min_distance_;
    int cfg_segm_min_inliers_;
    double cfg_switch_tolerance_;
    double cfg_line_cluster_tolerance_;
    double cfg_line_cluster_quota_;

    laser_geometry::LaserProjection laser_proj_;

    std::vector<LineInfo> last_lines_;

    // visualization
    size_t last_id_num_;
};


/** laser_lines node application.
 * @param argc argument count
 * @param argv array of arguments
 */
int
main(int argc, char **argv) {
  ros::init(argc, argv, "laser_lines");

  LaserLines laser_lines;

  ros::spin();
  return 0;
}
