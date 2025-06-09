#ifndef FX_LIDAR_UTILS_H
#define FX_LIDAR_UTILS_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "point_types.h"


namespace eroam {


/// Convert ROS PointCloud2 to PCL PointCloud
    inline CloudPtr PointCloud2ToCloudPtr(sensor_msgs::PointCloud2::Ptr msg) {
        CloudPtr cloud(new PointCloudType);
        pcl::fromROSMsg(*msg, *cloud);
        return cloud;
    }

/**
 * @brief Convert other types of point clouds to PointType point clouds.
 * Most commonly used to convert a full point cloud to an XYZI point cloud.
 * @tparam PointT The input point type.
 * @param input The input point cloud.
 * @return A shared pointer to the converted PointCloudType.
 */
    template<typename PointT = FullPointType>
    CloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input) {
        CloudPtr cloud(new PointCloudType);
        for (auto &pt: input->points) {
            PointType p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            cloud->points.template emplace_back(p);
        }
        cloud->width = input->width;
        return cloud;
    }

/// Voxel filter a point cloud with a specified resolution.
    inline CloudPtr VoxelCloud(CloudPtr cloud, float voxel_size = 0.1) {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel.setInputCloud(cloud);

        CloudPtr output(new PointCloudType);
        voxel.filter(*output);
        return output;
    }


}  // namespace eroam

#endif  // FX_LIDAR_UTILS_H