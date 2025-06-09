//
// Created by wlxing.
//

#ifndef EROAM_HPP
#define EROAM_HPP

#include "lidar_utils.h"
#include "eigen_types.h"
#include "math_utils.h"
#include "point_types.h"

#include "timer.h"
#include "ikd_Tree.h"

#include <deque>
#include <utility>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <dv_ros_msgs/EventArray.h>
#include <execution>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

namespace eroam
{
    using PointType = pcl::PointXYZI;                    // Point type used in the ikd-Tree.
    using PointVector = KD_TREE<PointType>::PointVector; // A vector of points for the ikd-Tree.

    /**
     * @class DVSIntrinsics
     * @brief Holds and manages the intrinsic parameters of the event camera (DVS).
     * This class is responsible for loading camera calibration parameters like the
     * intrinsic matrix and distortion coefficients from the ROS parameter server. It
     * stores them in both std::vector and cv::Mat formats for easy use with different libraries.
     */
    class DVSIntrinsics
    {
    public:
        int width_{0};
        int height_{0};

        std::vector<double> intrinsic_matrix_;
        std::vector<double> dist_params_;

        cv::Mat intrinsic_cvmat_;
        cv::Mat dist_cvmat_;

        DVSIntrinsics() = default;

        // Helper function (can be global or static member if preferred, or defined in a common utility header)
        template <typename T>
        std::string vec_to_string_for_dvs(const std::vector<T> &vec)
        {
            std::stringstream ss;
            ss << "[";
            for (size_t i = 0; i < vec.size(); ++i)
            {
                ss << vec[i];
                if (i < vec.size() - 1)
                {
                    ss << ", ";
                }
            }
            ss << "]";
            return ss.str();
        }

        /**
         * @brief Loads camera intrinsic and distortion parameters from the ROS parameter server.
         * @param nh_private A private ROS NodeHandle used to access node-specific parameters under the "dvs/" namespace.
         */
        void init_from_ros_params(ros::NodeHandle &nh_private)
        {
            // Retrieve DVS width and height.
            nh_private.param("dvs/width", width_, 0);
            nh_private.param("dvs/height", height_, 0);
            // ROS_INFO("DVSIntrinsics: Loaded width=%d, height=%d", width_, height_); // Printing will be done at the end

            // Retrieve the intrinsic matrix
            if (!nh_private.getParam("dvs/intrinsic_matrix", intrinsic_matrix_))
            {
                ROS_ERROR("DVSIntrinsics: Failed to get 'dvs/intrinsic_matrix' from parameter server.");
            }
            else if (intrinsic_matrix_.size() != 9)
            {
                ROS_WARN("DVSIntrinsics: 'dvs/intrinsic_matrix' size is %zu, expected 9. Clearing.", intrinsic_matrix_.size());
                intrinsic_matrix_.clear();
            }

            // Retrieve the distortion parameters
            if (!nh_private.getParam("dvs/dist_params", dist_params_))
            {
                ROS_ERROR("DVSIntrinsics: Failed to get 'dvs/dist_params' from parameter server.");
            }
            else if (dist_params_.size() != 5)
            {
                ROS_WARN("DVSIntrinsics: 'dvs/dist_params' size is %zu, expected 5. Clearing.", dist_params_.size());
                dist_params_.clear();
            }

            // Convert std::vector<double> to cv::Mat
            if (intrinsic_matrix_.size() == 9)
            {
                intrinsic_cvmat_ = cv::Mat(3, 3, CV_64FC1, intrinsic_matrix_.data()).clone();
            }
            else
            {
                ROS_ERROR("DVSIntrinsics: Cannot create intrinsic_cvmat_ due to incorrect size or missing intrinsic_matrix_.");
                // Consider setting intrinsic_cvmat_ to an empty Mat if it's not already.
                intrinsic_cvmat_ = cv::Mat();
            }

            if (dist_params_.size() == 5)
            {
                dist_cvmat_ = cv::Mat(5, 1, CV_64FC1, dist_params_.data()).clone();
            }
            else
            {
                ROS_ERROR("DVSIntrinsics: Cannot create dist_cvmat_ due to incorrect size or missing dist_params_.");
                dist_cvmat_ = cv::Mat();
            }

            // Print all loaded DVSIntrinsics parameters
            ROS_INFO("--- DVSIntrinsics Parameters Loaded ---");
            ROS_INFO_STREAM("  Width: " << width_);
            ROS_INFO_STREAM("  Height: " << height_);
            ROS_INFO_STREAM("  Intrinsic Matrix: " << vec_to_string_for_dvs(intrinsic_matrix_));
            ROS_INFO_STREAM("  Distortion Params: " << vec_to_string_for_dvs(dist_params_));
            ROS_INFO("--------------------------------------");
        }
    };

    /**
     * @class EROAM
     * @brief The main class for the Event-based Rotational Odometry and Mapping (EROAM) system.
     * This class encapsulates the entire pipeline, including event processing, pose estimation
     * via Event Spherical Iterative Closest Point (ES-ICP), and efficient map management using
     * an incremental k-d tree and Regional Density Management.
     */
    class EROAM
    {
    private:
        // --- Regional Density Management (RDM) ---
        // Described in Section III-E of the paper, this mechanism prevents the map from becoming
        // overly dense in frequently observed areas, ensuring stable, real-time performance.

        bool use_regional_density_management_ = true; // Flag to enable or disable RDM.

        // Defines a hash function for std::pair, used for the grid cell map.
        struct PairHash
        {
            template <class T1, class T2>
            size_t operator()(const std::pair<T1, T2> &p) const
            {
                return std::hash<T1>()(p.first) ^ std::hash<T2>()(p.second);
            }
        };

        // Maps a grid cell (latitude, longitude index) to the number of points it contains.
        std::unordered_map<std::pair<int, int>, int, PairHash> grid_cell_counter_;
        double rdm_cell_angle_ = 2.0; // Angular size of each grid cell in degrees.

        // A pre-calculated lookup table for the maximum point capacity of each latitude band.
        std::vector<int> latitude_cell_capacities_;
        int equator_cell_capacity_ = 30; // Base capacity for cells at the equator.
        int min_cell_capacity_ = 5;      // Minimum capacity to ensure sparse areas are still mapped.

    public:
        /**
         * @brief Initializes the parameters for Regional Density Management (RDM).
         * This function pre-calculates the point capacity for each latitude cell based on its
         * surface area, with cells near the poles having smaller capacities than those at the equator.
         * @param cell_angle The angular size (degrees) of each grid cell.
         * @param equator_capacity The maximum number of points allowed in a cell at the equator.
         * @param min_capacity The minimum capacity for any cell, ensuring even polar regions are mapped.
         */
        void InitRegionalDensityManagement(double cell_angle = 2.0, int equator_capacity = 30, int min_capacity = 5)
        {
            rdm_cell_angle_ = cell_angle;
            equator_cell_capacity_ = equator_capacity;
            min_cell_capacity_ = min_capacity;

            // Calculate number of latitude grid cells
            int num_lat_cells = static_cast<int>(180.0 / rdm_cell_angle_) + 1;

            // Precalculate maximum capacity for each latitude index
            latitude_cell_capacities_.resize(num_lat_cells);

            for (int lat_idx = 0; lat_idx < num_lat_cells; ++lat_idx)
            {
                // Calculate the latitude value at the center of this grid cell
                double grid_center_lat = (lat_idx * rdm_cell_angle_ + rdm_cell_angle_ / 2) - 90.0;
                double latitude_rad = grid_center_lat * M_PI / 180.0;

                // Adjust maximum capacity based on latitude (scales by cosine of latitude)
                int adjusted_capacity = static_cast<int>(equator_cell_capacity_ * std::cos(latitude_rad));
                latitude_cell_capacities_[lat_idx] = std::max(min_cell_capacity_, adjusted_capacity);
            }

            LOG(INFO) << "EROAM: Initialized regional density management with "
                      << num_lat_cells << " latitude cells, angle=" << rdm_cell_angle_
                      << " degree, equator_capacity=" << equator_cell_capacity_;
        }

        /**
         * @brief Calculates the (latitude, longitude) grid indices for a given 3D point on the sphere.
         * @param point A 3D point in the camera's coordinate system.
         * @return A std::pair of integer indices {latitude_index, longitude_index}.
         */
        std::pair<int, int> GetLatLongGridIndex(const Vec3d &point)
        {
            double r = point.norm();
            if (r < 1e-6)
                return {-1, -1}; // Handle special case of origin point

            // Event camera coordinate system: z forward, y down, x right
            double latitude = std::asin(-point.y() / r) * 180.0 / M_PI;         // Latitude (-90 degree,90 degree)
            double longitude = std::atan2(point.x(), point.z()) * 180.0 / M_PI; // Longitude (-180 degree,180 degree)

            int lat_idx = static_cast<int>((latitude + 90.0) / rdm_cell_angle_);
            int lon_idx = static_cast<int>((longitude + 180.0) / rdm_cell_angle_);

            return {lat_idx, lon_idx};
        }

        /**
         * @brief Retrieves the pre-calculated maximum point capacity for a given latitude index.
         * @param lat_idx The latitude index of the grid cell.
         * @return The maximum number of points allowed in that cell's latitude band.
         */
        int GetGridCapacity(int lat_idx)
        {
            if (lat_idx >= 0 && lat_idx < latitude_cell_capacities_.size())
            {
                return latitude_cell_capacities_[lat_idx];
            }
            return min_cell_capacity_; // Return minimum capacity for invalid indices
        }

        /**
         * @brief Filters a vector of new points according to the RDM policy before adding them to the map.
         * It checks each point against its corresponding grid cell's capacity, discarding points
         * that would exceed the density limit.
         * @param new_points The points from a new keyframe to be considered for addition to the map.
         * @return A PointVector containing only the points that passed the density filter.
         */
        PointVector ApplyRegionalDensityManagement(const PointVector &new_points)
        {
            PointVector filtered_points;

            for (const auto &point : new_points)
            {
                Vec3d pt = ToVec3d(point);

                // Get grid cell for this point
                auto grid_idx = GetLatLongGridIndex(pt);
                if (grid_idx.first == -1)
                    continue; // Skip invalid points

                // Get capacity for this latitude grid cell
                int cell_capacity = GetGridCapacity(grid_idx.first);

                // Check if grid cell has reached capacity
                if (grid_cell_counter_[grid_idx] < cell_capacity)
                {
                    filtered_points.push_back(point);
                    grid_cell_counter_[grid_idx]++;
                }
            }

            return filtered_points;
        }

    public:
        /**
         * @struct Options
         * @brief A struct to hold all configurable parameters for the EROAM system.
         * These parameters control the behavior of the algorithm, including ICP settings,
         * keyframe creation criteria, and event processing thresholds. They are typically
         * loaded from a ROS launch file.
         */
        struct Options
        {

            Options() = default;

            // In struct EROAM::Options
            // Helper function (can be global or static member if preferred, or defined in a common utility header)
            template <typename T>
            std::string vec_to_string_for_options(const std::vector<T> &vec)
            { // Renamed to avoid redefinition
                std::stringstream ss;
                ss << "[";
                for (size_t i = 0; i < vec.size(); ++i)
                {
                    ss << vec[i];
                    if (i < vec.size() - 1)
                    {
                        ss << ", ";
                    }
                }
                ss << "]";
                return ss.str();
            }

            void load_from_ros_params(ros::NodeHandle &nh_private)
            {
                // Load parameters from "options/" namespace
                nh_private.param("options/event_sphere_radius", this->event_sphere_radius_, 1.0);
                nh_private.param("options/max_frame_evs", this->max_frame_evs_, 3000);
                nh_private.param("options/min_initial_frame_evs", this->min_initial_frame_evs_, 200);
                nh_private.param("options/min_frame_evs", this->min_frame_evs_, 2000);
                nh_private.param("options/kf_angle_deg", this->kf_angle_deg_, 2.0);
                nh_private.param("options/max_iteration", this->max_iteration_, 4);
                nh_private.param("options/max_line_distance", this->max_line_distance_, 0.006);
                nh_private.param("options/min_effective_pts", this->min_effective_pts_, 10);
                nh_private.param("options/eps", this->eps_, 5e-6);

                // Print all loaded EROAM::Options parameters
                ROS_INFO("--- EROAM::Options Parameters Loaded ---");
                ROS_INFO_STREAM("  Event Sphere Radius: " << this->event_sphere_radius_);
                ROS_INFO_STREAM("  Max Frame Events: " << this->max_frame_evs_);
                ROS_INFO_STREAM("  Min Initial Frame Events: " << this->min_initial_frame_evs_);
                ROS_INFO_STREAM("  Min Frame Events: " << this->min_frame_evs_);
                ROS_INFO_STREAM("  Keyframe Angle (deg): " << this->kf_angle_deg_);
                ROS_INFO_STREAM("  Max Iteration (ICP): " << this->max_iteration_);
                ROS_INFO_STREAM("  Max Line Distance (ICP): " << this->max_line_distance_);
                ROS_INFO_STREAM("  Min Effective Points (ICP): " << this->min_effective_pts_);
                ROS_INFO_STREAM("  EPS (ICP Convergence): " << this->eps_);
                ROS_INFO("--------------------------------------");
            }
            // event sphere parameters
            double event_sphere_radius_ = 1.0; // Radius of the sphere onto which events are projected.

            int max_frame_evs_ = 1500;         // The number of events (n) to process in each frame.
            int min_initial_frame_evs_ = 1500; // Minimum events required to initialize the very first map frame.
            int min_frame_evs_ = 500;          // Minimum events required in a frame to be processed (discards sparse frames).

            double kf_angle_deg_ = 1; // Rotational threshold (in degrees) to trigger a new keyframe.

            // ICP parameters
            int max_iteration_ = 8;             // Max iterations for the Gauss-Newton optimization.
            double max_line_distance_ = 0.0005; // Max distance for point-to-line correspondence.
            int min_effective_pts_ = 30;        // Minimum number of inliers for a valid alignment.
            double eps_ = 0.000005;             // Convergence threshold for the ICP update step (dx).
        };

        EROAM() = default;

        ~EROAM()
        {
            LOG(INFO) << "EROAM: saving files.";
            eroam::common::Timer::Evaluate(
                [this]()
                { this->WritePanoramic(); }, "EROAM: saving panoramic image");

            LOG(INFO) << "EROAM: saving pose result to file: " << pose_result_file_;

            eroam::common::Timer::Evaluate(
                [this]()
                { this->WriteResult(); }, "EROAM: saving pose result");

            LOG(INFO) << "EROAM: done.";

            eroam::common::Timer::PrintAll();
        }

        CloudPtr GenerateCameraFrame(const SE3 &pose)
        {
            CloudPtr camera_frame(new PointCloudType);

            // virtual camera K
            double fx = 200.0;
            double fy = 200.0;
            double cx = 120.0;
            double cy = 90.0;
            double width = 240;
            double height = 180;
            double virtual_camera_distance = 0.2;
            int num_points = 50; // points on each edge

            // camera plane
            Vec3d p0_uv(0, 0, 0);
            Vec3d p1_uv(width, 0, 0);
            Vec3d p2_uv(width, height, 0);
            Vec3d p3_uv(0, height, 0);

            // camera small triangle
            Vec3d p4_uv(width, -height * 0.07, 0);
            Vec3d p5_uv(0, -height * 0.07, 0);
            Vec3d p6_uv(width / 2, -height * 0.25, 0);

            // normalize
            auto to_normalized = [&](const Vec3d &p)
            {
                return Vec3d((p.x() - cx) / fx, (p.y() - cy) / fy, 1.0);
            };

            Vec3d p0_n = to_normalized(p0_uv);
            Vec3d p1_n = to_normalized(p1_uv);
            Vec3d p2_n = to_normalized(p2_uv);
            Vec3d p3_n = to_normalized(p3_uv);
            Vec3d p4_n = to_normalized(p4_uv);
            Vec3d p5_n = to_normalized(p5_uv);
            Vec3d p6_n = to_normalized(p6_uv);

            // pose transformation
            Vec3d p0_transformed = pose * p0_n;
            Vec3d p1_transformed = pose * p1_n;
            Vec3d p2_transformed = pose * p2_n;
            Vec3d p3_transformed = pose * p3_n;
            Vec3d p4_transformed = pose * p4_n;
            Vec3d p5_transformed = pose * p5_n;
            Vec3d p6_transformed = pose * p6_n;

            // virtual points pose
            Vec3d p0_virtual = p0_transformed * virtual_camera_distance;
            Vec3d p1_virtual = p1_transformed * virtual_camera_distance;
            Vec3d p2_virtual = p2_transformed * virtual_camera_distance;
            Vec3d p3_virtual = p3_transformed * virtual_camera_distance;
            Vec3d p4_virtual = p4_transformed * virtual_camera_distance;
            Vec3d p5_virtual = p5_transformed * virtual_camera_distance;
            Vec3d p6_virtual = p6_transformed * virtual_camera_distance;

            // edges
            std::vector<std::pair<Vec3d, Vec3d>> edges = {
                {p0_virtual, p1_virtual},
                {p1_virtual, p2_virtual},
                {p2_virtual, p3_virtual},
                {p3_virtual, p0_virtual},
                {p4_virtual, p5_virtual},
                {p5_virtual, p6_virtual},
                {p6_virtual, p4_virtual},
                {Vec3d(0, 0, 0), p0_virtual},
                {Vec3d(0, 0, 0), p1_virtual},
                {Vec3d(0, 0, 0), p2_virtual},
                {Vec3d(0, 0, 0), p3_virtual}};

            // points on edges
            for (const auto &edge : edges)
            {
                const Vec3d &start = edge.first;
                const Vec3d &end = edge.second;

                for (int i = 0; i <= num_points; ++i)
                {
                    double t = static_cast<double>(i) / num_points;
                    Vec3d point = start + t * (end - start);
                    camera_frame->push_back(ToPointType(point));
                }
            }

            return camera_frame;
        }

        void WriteResult()
        {
            std::ofstream ofs(pose_result_file_);
            if (!ofs.is_open())
            {
                LOG(ERROR) << "EROAM: failed to open pose result file: " << pose_result_file_;
                return;
            }

            for (int i = 0; i < estimated_poses_se3_.size(); ++i)
            {
                ofs << std::fixed << std::setprecision(6)
                    << estimated_poses_ts_[i].toSec() << " "
                    << estimated_poses_se3_[i].translation().transpose()[0] << " "
                    << estimated_poses_se3_[i].translation().transpose()[1] << " "
                    << estimated_poses_se3_[i].translation().transpose()[2] << " "
                    << estimated_poses_se3_[i].so3().unit_quaternion().coeffs().transpose()[0] << " "
                    << estimated_poses_se3_[i].so3().unit_quaternion().coeffs().transpose()[1] << " "
                    << estimated_poses_se3_[i].so3().unit_quaternion().coeffs().transpose()[2] << " "
                    << estimated_poses_se3_[i].so3().unit_quaternion().coeffs().transpose()[3] << std::endl;
            }
            ofs.close();
        }

        /**
         * @brief Generates and saves a panoramic image from all accumulated event frames.
         * This function projects all points from the spherical representation onto a virtual
         * cylinder, which is then "unrolled" to create a 2D panoramic image, as illustrated
         * in Section III-F of the paper.
         */
        void WritePanoramic()
        {
            // Collect all point clouds into a single cloud
            CloudPtr global_map_event_sphere = nullptr;
            global_map_event_sphere.reset(new PointCloudType);
            for (auto cloud : global_map_event_sphere_vec_)
            {
                *global_map_event_sphere += *cloud;
            }

            // Basic validation
            if (this->panoramic_output_file_.empty())
            {
                ROS_WARN("EROAM::WritePanoramic: Output file path for panoramic image is empty. Skipping saving.");
                return;
            }

            if (!global_map_event_sphere || global_map_event_sphere->empty())
            {
                ROS_WARN("EROAM::WritePanoramic: Global map event sphere is null or empty. Skipping panoramic image saving.");
                return;
            }

            CloudPtr global_map_event_sphere_vis(new PointCloudType);
            Eigen::Isometry3d vis_transform = Eigen::Isometry3d::Identity();
            vis_transform.rotate(Eigen::AngleAxisd(3 * M_PI / 2, Eigen::Vector3d::UnitX()));
            // vis_transform.rotate(Eigen::AngleAxisd(M_PI / 180.0 * 15, Eigen::Vector3d::UnitX()));

            pcl::transformPointCloud(*global_map_event_sphere, *global_map_event_sphere_vis, vis_transform.matrix());

            // Get panoramic settings parameters
            int height_res = this->panoramic_height_res_;
            double cylinder_radius = options_.event_sphere_radius_;

            // Process longitude range, only ensuring start < end
            double lon_start = this->panoramic_longitude_start_;
            double lon_end = this->panoramic_longitude_end_;

            if (lon_start > lon_end)
            {
                std::swap(lon_start, lon_end);
                LOG(INFO) << "EROAM::WritePanoramic: Swapped longitude start/end values to ensure start < end";
            }

            // Calculate latitude range
            double lat_start = std::max(-90.0, std::min(90.0, this->panoramic_latitude_start_));
            double lat_end = std::max(-90.0, std::min(90.0, this->panoramic_latitude_end_));
            if (lat_start > lat_end)
            {
                std::swap(lat_start, lat_end);
                LOG(INFO) << "EROAM::WritePanoramic: Swapped latitude start/end values to ensure start < end";
            }
            double latitude_range = lat_end - lat_start;

            // Parameter validation
            if (cylinder_radius <= 0 || height_res <= 0 || latitude_range <= 0)
            {
                ROS_ERROR("EROAM::WritePanoramic: Invalid parameters for panoramic projection (radius: %f, height_res: %d, "
                          "latitude_range: %f). Skipping.",
                          cylinder_radius, height_res, latitude_range);
                return;
            }

            // Calculate cylinder height based on actual latitude range
            double lat_start_rad = lat_start * math::kDEG2RAD;
            double lat_end_rad = lat_end * math::kDEG2RAD;

            // Correction: Calculate the actual cylinder height difference
            double cylinder_height = cylinder_radius * (tan(lat_end_rad) - tan(lat_start_rad));

            if (cylinder_height <= 0)
            {
                ROS_ERROR("EROAM::WritePanoramic: Calculated cylinder height is non-positive (%f). Skipping.", cylinder_height);
                return;
            }

            // Calculate single loop image width for complete 360 degrees
            double cylinder_perimeter = 2 * M_PI * cylinder_radius;
            int single_loop_width = static_cast<int>(cylinder_perimeter / cylinder_height * height_res);

            if (single_loop_width <= 0)
            {
                ROS_ERROR("EROAM::WritePanoramic: Calculated single loop width is non-positive (%d). Skipping.", single_loop_width);
                return;
            }

            // First create a complete 360-degree panorama
            cv::Mat full_loop_img = cv::Mat::zeros(height_res, single_loop_width, CV_8UC1);
            cv::Mat full_loop_cnt = cv::Mat::zeros(height_res, single_loop_width, CV_64FC1);

            // Lambda function to map points to full panoramic coordinates
            auto ToFullPanoramic = [&](const Vec3d &point) -> std::vector<int>
            {
                Vec3d point_on_cylinder = point;
                double xy_radius_sq = point_on_cylinder.x() * point_on_cylinder.x() +
                                      point_on_cylinder.y() * point_on_cylinder.y();
                if (xy_radius_sq < 1e-9)
                {
                    return std::vector<int>{-1, -1}; // Invalid point
                }

                double xy_radius = sqrt(xy_radius_sq);
                double rate = cylinder_radius / xy_radius;
                point_on_cylinder.x() *= rate;
                point_on_cylinder.y() *= rate;
                point_on_cylinder.z() *= rate;

                // Calculate longitude and latitude
                double theta = atan2(point_on_cylinder.y(), point_on_cylinder.x()); // Range: [-PI, PI]
                double phi = atan2(point_on_cylinder.z(), cylinder_radius);         // Related to latitude

                // Convert to degrees
                double theta_deg = theta * 180.0 / M_PI; // Range: [-180, 180]
                double phi_deg = phi * 180.0 / M_PI;

                // Check latitude range
                if (phi_deg < lat_start || phi_deg > lat_end)
                {
                    return std::vector<int>{-1, -1}; // Outside latitude range
                }

                // Map to single-loop image coordinates (theta_deg range is [-180,180])
                // Convert theta from [-180,180] to [0,360] for easier processing
                if (theta_deg < 0)
                {
                    theta_deg += 360.0;
                }

                int x = static_cast<int>(theta_deg / 360.0 * single_loop_width);
                int y = static_cast<int>((phi_deg - lat_start) / latitude_range * height_res);

                // Apply axis inversions to maintain consistency
                y = height_res - 1 - y;        // Invert Y-axis, higher latitude at top
                x = single_loop_width - 1 - x; // Invert X-axis as requested

                // Ensure coordinates are within image boundaries
                if (x < 0)
                    x = 0;
                if (x >= single_loop_width)
                    x = single_loop_width - 1;
                if (y < 0 || y >= height_res)
                {
                    return std::vector<int>{-1, -1}; // Outside Y-axis range
                }

                return std::vector<int>{x, y};
            };

            // Project all points to the single-loop panorama
            for (const auto &point : *global_map_event_sphere_vis)
            {
                std::vector<int> xy = ToFullPanoramic(ToVec3d(point));
                if (xy[0] < 0)
                {
                    continue; // Skip invalid points
                }
                full_loop_cnt.at<double>(xy[1], xy[0]) += 1;
            }

            // Process single-loop image brightness
            std::vector<double> cnt_values;
            for (int i = 0; i < full_loop_img.rows; ++i)
            {
                for (int j = 0; j < full_loop_img.cols; ++j)
                {
                    if (full_loop_cnt.at<double>(i, j) > 0)
                    {
                        cnt_values.emplace_back(full_loop_cnt.at<double>(i, j));
                    }
                }
            }

            if (cnt_values.empty())
            {
                ROS_WARN("EROAM::WritePanoramic: No points were projected onto the panoramic image. Skipping image generation.");
                return;
            }

            // Calculate brightness threshold
            std::sort(cnt_values.begin(), cnt_values.end(), std::greater<double>());
            size_t top_idx = static_cast<size_t>(cnt_values.size() * this->panoramic_top_brightness_percent_);
            if (top_idx >= cnt_values.size())
                top_idx = cnt_values.size() - 1;
            if (top_idx < 0 && !cnt_values.empty())
                top_idx = 0;

            double cnt_value_top_thresh = cnt_values[top_idx];
            if (cnt_value_top_thresh < 1e-6)
                cnt_value_top_thresh = 1.0;

            // Generate single-loop panorama
            for (int i = 0; i < full_loop_img.rows; ++i)
            {
                for (int j = 0; j < full_loop_img.cols; ++j)
                {
                    double cnt_pixel = full_loop_cnt.at<double>(i, j);
                    cnt_pixel = cnt_pixel > cnt_value_top_thresh ? cnt_value_top_thresh : cnt_pixel;
                    double cnt_rate = (cnt_pixel * cnt_pixel) / (cnt_value_top_thresh * cnt_value_top_thresh);
                    double value = cnt_rate * 255.0;
                    value = std::max(0.0, std::min(255.0, value));
                    full_loop_img.at<uchar>(i, j) = static_cast<uchar>(255 - static_cast<uchar>(value));
                }
            }

            // Now, create the final panorama based on longitude range
            // Calculate total longitude range and corresponding pixel width
            double longitude_range = lon_end - lon_start;
            int final_width = static_cast<int>(longitude_range / 360.0 * single_loop_width);

            // If the image is too large, issue a warning but continue
            if (final_width > 20000)
            {
                LOG(WARNING) << "EROAM::WritePanoramic: Final image width is very large (" << final_width
                             << " pixels). This may cause memory issues.";
            }

            // Create the final image
            cv::Mat panoramic_img = cv::Mat::zeros(height_res, final_width, CV_8UC1);

            // Directly copy pixels from the single loop panorama to the final image
            // The single loop panorama represents 0-360 degrees
            for (int x = 0; x < final_width; ++x)
            {
                // Calculate which longitude degree this pixel corresponds to
                double target_lon = lon_start + (x / static_cast<double>(final_width)) * longitude_range;

                // Normalize to 0-360 range for indexing into the single loop panorama
                double normalized_lon = fmod(target_lon, 360.0);
                if (normalized_lon < 0)
                    normalized_lon += 360.0;

                // Calculate the source pixel in the single loop panorama
                // Note: X-axis is already inverted in the single loop panorama
                int src_x = static_cast<int>(normalized_lon / 360.0 * single_loop_width);

                // Ensure source pixel is within bounds
                if (src_x < 0)
                    src_x = 0;
                if (src_x >= single_loop_width)
                    src_x = single_loop_width - 1;

                // Direct pixel copy for each row (no interpolation)
                for (int y = 0; y < height_res; ++y)
                {
                    panoramic_img.at<uchar>(y, x) = full_loop_img.at<uchar>(y, src_x);
                }
            }

            // Apply width ratio cropping (if specified)
            cv::Mat panoramic_img_center;

            panoramic_img_center = panoramic_img.clone();

            // Save PNG version
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
            if (!cv::imwrite(this->panoramic_output_file_, panoramic_img_center, compression_params))
            {
                LOG(ERROR) << "EROAM::WritePanoramic: Failed to write PNG panoramic image to: " << this->panoramic_output_file_;
            }
            else
            {
                LOG(INFO) << "EROAM: PNG panoramic image saved to: " << this->panoramic_output_file_
                          << " (longitude: " << lon_start << " to " << lon_end
                          << " degree [range: " << longitude_range << " degree, "
                          << (longitude_range / 360.0) << " loop(s)], latitude: "
                          << lat_start << " to " << lat_end << " degree, cylinder_height: " << cylinder_height
                          << ", single_loop_width: " << single_loop_width << ")";

                // Also save JPEG version
                compression_params.clear();
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(70);

                std::string jpeg_file = this->panoramic_output_file_;
                size_t dot_pos = jpeg_file.rfind('.');
                if (dot_pos != std::string::npos)
                {
                    jpeg_file.replace(dot_pos, std::string::npos, ".jpg");
                }
                else
                {
                    jpeg_file += ".jpg";
                }

                if (!cv::imwrite(jpeg_file, panoramic_img_center, compression_params))
                {
                    LOG(ERROR) << "EROAM::WritePanoramic: Failed to write JPEG panoramic image to: " << jpeg_file;
                }
                else
                {
                    LOG(INFO) << "EROAM: JPEG panoramic image saved to: " << jpeg_file;
                }
            }
        }

        /**
         * @brief Initializes the entire EROAM system.
         * This involves setting up ROS subscribers and publishers, loading all necessary
         * parameters from the launch file, and pre-calculating the undistortion map for the
         * event camera to enable fast event projection.
         * @param nh A public ROS NodeHandle for topic communication.
         * @return True if initialization is successful, false otherwise.
         */
        bool Init(ros::NodeHandle &nh) // 'nh' is the public NodeHandle
        {

            // Create a private NodeHandle ("~") to access this node's private parameters.
            ros::NodeHandle nh_private("~");

            std::string ev_topic_path; // Local variable to hold event topic string

            // Load parameters using their new namespaced paths from the launch file.

            // DVS specific parameters (ev_topic is now under 'dvs/')
            // Note: dvs_intrinsics_ (width, height, K, D) are loaded by dvs_intrinsics_.init_from_ros_params()
            nh_private.param<std::string>("dvs/ev_topic", ev_topic_path, "/dvs/events"); // Default from your HPP
            nh_private.param<double>("dvs/dvxplorer_correction_ratio", this->dvxplorer_correction_ratio_, 1.0);

            // Output specific parameters
            nh_private.param<std::string>("output/pose_result_file", this->pose_result_file_, "/tmp/pose_result.txt");         // Default from your HPP
            nh_private.param<std::string>("output/panoramic_output_file", this->panoramic_output_file_, "/tmp/panoramic.png"); // Default from your HPP
            nh_private.param<int>("output/panoramic_height_res", this->panoramic_height_res_, 1000);                           // Default from your HPP
            nh_private.param<double>("output/panoramic_top_brightness_percent", this->panoramic_top_brightness_percent_, 0.3); // Default from your HPP
            nh_private.param("output/panoramic_longitude_start", this->panoramic_longitude_start_, -180.0);
            nh_private.param("output/panoramic_longitude_end", this->panoramic_longitude_end_, 180.0);
            nh_private.param("output/panoramic_latitude_start", this->panoramic_latitude_start_, -90.0);
            nh_private.param("output/panoramic_latitude_end", this->panoramic_latitude_end_, 90.0);
            // Debug specific parameters
            nh_private.param<bool>("debug/save_frames", this->save_frames_, false); // Default from your HPP

            // Load regional density management parameters
            nh_private.param<bool>("rdm/use_regional_density_management", this->use_regional_density_management_, true);
            double cell_angle = 2.0;
            int equator_capacity = 150;
            int min_capacity = 5;
            nh_private.param<double>("rdm/cell_angle", cell_angle, 2.0);
            nh_private.param<int>("rdm/equator_capacity", equator_capacity, 150);
            nh_private.param<int>("rdm/min_capacity", min_capacity, 5);
            // Initialize regional density management based on parameters
            if (this->use_regional_density_management_)
            {
                InitRegionalDensityManagement(cell_angle, equator_capacity, min_capacity);
            }

            // Initialize Options and DVSIntrinsics from ROS parameters
            // These methods will use their own "options/" and "dvs/" prefixes internally
            // and print their loaded parameters.
            options_.load_from_ros_params(nh_private);
            dvs_intrinsics_.init_from_ros_params(nh_private);

            // Subscribe to event topic (using public NodeHandle as topics are global)
            ev_sub_ = nh.subscribe<dv_ros_msgs::EventArray>(ev_topic_path, 200000,
                                                            [this](const dv_ros_msgs::EventArray::ConstPtr &msg)
                                                            {
                                                                eroam::common::Timer::Evaluate(
                                                                    [&, this]()
                                                                    {
                                                                        this->ProcessEventStream(msg);
                                                                    },
                                                                    "EROAM: full time");
                                                            });

            // --- Initialize ikd-Tree ---
            float ikd_delete_param = 0.5f;
            float ikd_balance_param = 0.6f;
            double ikd_box_length = 0.0001;
            ikdtree_sphere_ = std::make_shared<KD_TREE<PointType>>(ikd_delete_param, ikd_balance_param, ikd_box_length);

            if (!ikdtree_sphere_)
            {
                LOG(ERROR) << "EROAM::Init: Failed to initialize ikd-tree!";
                return false;
            }
            // LOG(INFO) is now part of the final printout block

            // --- Initialize precomputed_undistortion_vec_ ---
            if (dvs_intrinsics_.width_ > 0 && dvs_intrinsics_.height_ > 0 &&
                !dvs_intrinsics_.intrinsic_cvmat_.empty() && !dvs_intrinsics_.dist_cvmat_.empty())
            {
                precomputed_undistortion_vec_.assign(dvs_intrinsics_.width_,
                                                     std::vector<Vec3d>(dvs_intrinsics_.height_, Vec3d(0, 0, 0)));
                std::vector<cv::Point2f> distorted_pixels, undistorted_normalized_coords;
                distorted_pixels.reserve(dvs_intrinsics_.width_ * dvs_intrinsics_.height_); // Pre-allocate
                for (int i = 0; i < dvs_intrinsics_.width_; ++i)
                {
                    for (int j = 0; j < dvs_intrinsics_.height_; ++j)
                    {
                        distorted_pixels.emplace_back(static_cast<float>(i), static_cast<float>(j));
                    }
                }
                cv::undistortPoints(distorted_pixels, undistorted_normalized_coords,
                                    dvs_intrinsics_.intrinsic_cvmat_,
                                    dvs_intrinsics_.dist_cvmat_,
                                    cv::noArray(),  // No rectification transform R
                                    cv::noArray()); // No new camera matrix P, results in normalized coordinates

                for (size_t i = 0; i < undistorted_normalized_coords.size(); ++i)
                {
                    Vec3d undistorted_point_on_plane(undistorted_normalized_coords[i].x, undistorted_normalized_coords[i].y, 1.0);
                    undistorted_point_on_plane.normalize();
                    undistorted_point_on_plane *= options_.event_sphere_radius_; // From EROAM::Options

                    int u = static_cast<int>(distorted_pixels[i].x);
                    int v = static_cast<int>(distorted_pixels[i].y);
                    // Bounds check though emplace_back should ensure distorted_pixels[i] is valid for u,v
                    if (u >= 0 && u < dvs_intrinsics_.width_ && v >= 0 && v < dvs_intrinsics_.height_)
                    {
                        precomputed_undistortion_vec_[u][v] = undistorted_point_on_plane;
                    }
                }
                // LOG(INFO) is now part of the final printout block
            }
            else
            {
                LOG(ERROR) << "EROAM::Init: DVS intrinsics not properly loaded for undistortion map generation (width/height invalid or K/D matrices empty).";
                return false;
            }
            // --- End Precomputed Undistortion Vec Initialization ---

            // Initialize publishers (using public NodeHandle).
            ev_sphere_frame_pub_ = nh.advertise<sensor_msgs::PointCloud2>("ev_sphere_frame", 100);
            map_frame_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_frame", 100);
            local_map_event_sphere_pub_ = nh.advertise<sensor_msgs::PointCloud2>("local_map_event_sphere",
                                                                                 100);
            edge_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("edge_pc", 100);
            surf_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("surf_pc", 100);
            pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("lidar_pose", 100);
            path_pub_ = nh.advertise<nav_msgs::Path>("lidar_path", 100);
            camera_frame_pub_ = nh.advertise<sensor_msgs::PointCloud2>("camera_frame", 100);

            // Print all EROAM Init-specific parameters loaded directly here
            ROS_INFO("--- EROAM::Init Direct Parameters Loaded ---");
            ROS_INFO_STREAM("  DVS Event Topic: " << ev_topic_path);
            ROS_INFO_STREAM("  DVXplorer Correction Ratio: " << this->dvxplorer_correction_ratio_);
            ROS_INFO_STREAM("  Output Pose Result File: " << this->pose_result_file_);
            ROS_INFO_STREAM("  Output Panoramic File: " << this->panoramic_output_file_);
            ROS_INFO_STREAM("  Output Panoramic Height Res: " << this->panoramic_height_res_);
            ROS_INFO_STREAM("  Output Panoramic Top Brightness: " << this->panoramic_top_brightness_percent_);
            ROS_INFO_STREAM("  Output Panoramic Longitude Range: [" << this->panoramic_longitude_start_
                                                                    << "degree, " << this->panoramic_longitude_end_ << " degree]");
            ROS_INFO_STREAM("  Output Panoramic Latitude Range: [" << this->panoramic_latitude_start_
                                                                   << " degree, " << this->panoramic_latitude_end_ << " degree]");
            ROS_INFO_STREAM("  Debug Save Frames: " << (this->save_frames_ ? "true" : "false"));
            ROS_INFO("--------------------------------------");
            ROS_INFO("--- IKD-Tree Initialization ---");
            ROS_INFO_STREAM("  IKD-Tree Delete Param: " << ikd_delete_param);
            ROS_INFO_STREAM("  IKD-Tree Balance Param: " << ikd_balance_param);
            ROS_INFO_STREAM("  IKD-Tree Box Length: " << ikd_box_length);
            ROS_INFO("--------------------------------------");
            ROS_INFO("--- Regional Density Management ---");
            ROS_INFO_STREAM("  Use Regional Density Management: " << (this->use_regional_density_management_ ? "true" : "false"));
            ROS_INFO_STREAM("  Cell Angle: " << cell_angle);
            ROS_INFO_STREAM("  Equator Cell Capacity: " << equator_capacity);
            ROS_INFO_STREAM("  Minimum Cell Capacity: " << min_capacity);
            ROS_INFO("--------------------------------------");
            ROS_INFO("--- Undistortion Map Generation ---");
            if (dvs_intrinsics_.width_ > 0 && dvs_intrinsics_.height_ > 0 && !dvs_intrinsics_.intrinsic_cvmat_.empty() && !dvs_intrinsics_.dist_cvmat_.empty())
            {
                ROS_INFO("  Precomputed undistortion map generated successfully.");
            }
            else
            {
                ROS_INFO("  Precomputed undistortion map generation FAILED.");
            }
            ROS_INFO("--------------------------------------");
            LOG(INFO) << "EROAM::Init: Initialization complete.";
            return true;
        }

        /**
         * @brief The main callback function for processing incoming event streams from the camera.
         * This function is the entry point for all real-time processing. It takes a batch of events,
         * forms a spherical frame, performs motion compensation, and triggers the ES-ICP alignment.
         * @param msg A constant pointer to the received dv_ros_msgs::EventArray message.
         */
        void ProcessEventStream(const dv_ros_msgs::EventArray::ConstPtr &msg)
        {
            static std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now(); // system start time

            // get the current time
            auto current_time = std::chrono::steady_clock::now();
            double time_diff =
                std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.0;

            LOG(INFO) << "--------------------------------------" << std::endl;
            LOG(INFO) << "EROAM: system runtime: " << std::fixed << std::setprecision(3)
                      << time_diff << " seconds" << std::endl;

            if (msg->events.empty())
            {
                return;
            }

            LOG(INFO) << "EROAM: processing event frame id:" << cnt_frame_++ << ", frame start time: "
                      << msg->events[0].ts;

            if (msg->events.size() < options_.min_frame_evs_)
            {
                return;
            }

            // undistort the event stream to sphere
            CloudPtr ev_sphere_frame(new PointCloudType);
            ev_sphere_frame->reserve(msg->events.size());
            std::vector<ros::Time> ev_shpere_frame_ts;
            ev_shpere_frame_ts.reserve(msg->events.size());

            // Determine how many events to consider and how to sample them
            std::vector<int> ev_idx;
            int ev_size = 0;

            if (msg->events.size() <= options_.max_frame_evs_)
            {
                // Case 1: Fewer events than max_frame_evs_, take all events
                ev_size = msg->events.size();
                ev_idx.resize(ev_size);
                std::iota(ev_idx.begin(), ev_idx.end(), 0);
            }
            else if (msg->events.size() > options_.max_frame_evs_ * dvxplorer_correction_ratio_)
            {
                // Case 3: Many more events than the threshold
                // Take max_frame_evs_ samples from the first max_frame_evs_ * dvxplorer_correction_ratio_ events
                int total_events_to_consider = static_cast<int>(options_.max_frame_evs_ * dvxplorer_correction_ratio_);
                ev_size = options_.max_frame_evs_;
                ev_idx.resize(ev_size);

                // Calculate sampling step (how many input events correspond to one output event)
                double step = static_cast<double>(total_events_to_consider) / ev_size;

                for (int i = 0; i < ev_size; ++i)
                {
                    // Calculate index in the original array
                    int idx = static_cast<int>(i * step);
                    ev_idx[i] = idx;
                }
            }
            else
            {
                // Case 2: More events than max_frame_evs_ but fewer than the threshold,
                // take the first max_frame_evs_ events
                ev_size = options_.max_frame_evs_;
                ev_idx.resize(ev_size);
                std::iota(ev_idx.begin(), ev_idx.end(), 0);
            }

            ros::Time ev_frame_bg_ts = msg->events[ev_idx[0]].ts;
            ros::Time ev_frame_ed_ts = msg->events[ev_idx[ev_size - 1]].ts;
            ros::Time ev_frame_md_ts = ev_frame_bg_ts + (ev_frame_ed_ts - ev_frame_bg_ts) * 0.5;

            LOG(INFO) << "Event frame duration: " << (ev_frame_ed_ts - ev_frame_bg_ts).toSec() << " s";

            eroam::common::Timer::Evaluate(
                [&, this]()
                {
                    std::for_each(std::execution::seq, ev_idx.begin(), ev_idx.end(),
                                  [&](const int &i)
                                  {
                                      Vec3d ev_sphere_point = precomputed_undistortion_vec_[msg->events[i].x][msg->events[i].y];
                                      ev_sphere_frame->push_back(
                                          ToPointType(ev_sphere_point));
                                      ev_shpere_frame_ts.emplace_back(msg->events[i].ts);
                                  });
                },
                "EROAM: undistortion and projection");

            // ev_sphere_frame warp
            eroam::common::Timer::Evaluate(
                [&, this]()
                {
                    if (estimated_poses_se3_.size() > 5)
                    {
                        SE3 pose_t_1 = estimated_poses_se3_[estimated_poses_se3_.size() - 1];
                        SE3 pose_t_2 = estimated_poses_se3_[estimated_poses_se3_.size() - 2];
                        ros::Time ts_t_1 = estimated_poses_ts_[estimated_poses_ts_.size() - 1];
                        ros::Time ts_t_2 = estimated_poses_ts_[estimated_poses_ts_.size() - 2];

                        SO3 r_t_1_t_2 = pose_t_1.so3() * pose_t_2.inverse().so3();
                        Eigen::AngleAxisd r_vec(r_t_1_t_2.matrix());

                        double frame_time_diff = (ts_t_1 - ts_t_2).toSec();
                        if (std::abs(frame_time_diff) < 1e-6)
                        {
                            LOG(WARNING) << "EROAM: ev_sphere_frame warp - Frame time difference too small ("
                                         << frame_time_diff << " sec), skipping warp operation.";
                            return;
                        }

                        double rotation_rate = r_vec.angle() / frame_time_diff;

                        std::vector<int> indices(ev_sphere_frame->size());
                        std::iota(indices.begin(), indices.end(), 0);

                        std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
                                      [&](int i)
                                      {
                                          Vec3d point = ToVec3d(ev_sphere_frame->points[i]);

                                          double delta_t = (ev_shpere_frame_ts[i] - ev_frame_md_ts).toSec();
                                          double rotation_angle = rotation_rate * delta_t;

                                          Eigen::Matrix3d r_mat_per_point = Eigen::AngleAxisd(rotation_angle, r_vec.axis()).matrix();
                                          Vec3d point_rotated = r_mat_per_point * point;
                                          ev_sphere_frame->points[i] = ToPointType(point_rotated);
                                      });
                    }
                },
                "EROAM: ev_sphere_frame warp");

            // first frame
            if (local_map_event_sphere_ == nullptr)
            {
                if (msg->events.size() > options_.min_initial_frame_evs_)
                {
                    local_map_event_sphere_ = ev_sphere_frame;
                    PointVector first_frame_vec = ev_sphere_frame->points;
                    // build ikdtree
                    eroam::common::Timer::Evaluate(
                        [&]()
                        {
                            ikdtree_sphere_->Build(first_frame_vec);
                        },
                        "EROAM: ikd-Tree initial build");
                    LOG(INFO) << "EROAM: ikd-Tree built. Initial size: " << ikdtree_sphere_->size();
                    local_event_sphere_frames_.emplace_back(ev_sphere_frame);

                    estimated_poses_se3_.emplace_back(SE3());
                    estimated_poses_ts_.emplace_back(ev_frame_bg_ts);
                }
                return;
            }

            // spherical icp
            SE3 pose_se3;

            eroam::common::Timer::Evaluate(
                [&, this]()
                {
                    pose_se3 = AlignWithLocalSphere(ev_sphere_frame);
                },
                "EROAM: event spherical icp");

            // LOG(INFO) << "eroam: pose_se3: " << pose_se3.matrix();
            estimated_poses_se3_.emplace_back(pose_se3);
            estimated_poses_ts_.emplace_back(ev_frame_md_ts);

            // for visualization, rotate the event sphere make sure polar line is along z axis
            CloudPtr event_sphere_frame_world(new PointCloudType), event_sphere_frame_world_vis(new PointCloudType), map_frame_world(new PointCloudType);
            Eigen::Isometry3d vis_transform = Eigen::Isometry3d::Identity();
            vis_transform.rotate(Eigen::AngleAxisd(3 * M_PI / 2, Eigen::Vector3d::UnitX()));

            pcl::transformPointCloud(*ev_sphere_frame, *event_sphere_frame_world, pose_se3.matrix());
            pcl::transformPointCloud(*event_sphere_frame_world, *event_sphere_frame_world_vis, vis_transform.matrix());

            bool iskeyframe = false;
            if (IsKeyframe(pose_se3))
            {
                iskeyframe = true;
                static int kdtree_build_cnt = 0;
                kdtree_build_cnt++;

                last_kf_pose_ = pose_se3;
                last_kf_id_ = cnt_frame_;

                local_event_sphere_frames_.emplace_back(event_sphere_frame_world);

                int added_points_count = 0;
                PointVector current_frame_vec = event_sphere_frame_world->points;
                PointVector filtered_points;
                if (!current_frame_vec.empty())
                {
                    eroam::common::Timer::Evaluate(
                        [&]()
                        {
                            PointVector current_frame_vec = event_sphere_frame_world->points;
                            if (!current_frame_vec.empty())
                            {
                                if (use_regional_density_management_)
                                {
                                    filtered_points = ApplyRegionalDensityManagement(current_frame_vec);
                                }
                                else
                                {
                                    filtered_points = current_frame_vec; // No filtering
                                }
                                // Add to ikd-tree
                                added_points_count = ikdtree_sphere_->Add_Points(filtered_points, false);
                            }
                        },
                        "EROAM: ikd-tree update");
                    LOG(INFO) << "EROAM: ikdtree update " << current_frame_vec.size() << " raw points to ikd-tree,"
                              << " effective points: " << added_points_count
                              << ", tree total size: " << ikdtree_sphere_->size()
                              << ", valid points: " << ikdtree_sphere_->validnum();
                    eroam::common::Timer::Evaluate(
                        [&]()
                        {
                            map_frame_world->clear();
                            map_frame_world->resize(filtered_points.size());
                            std::copy(filtered_points.begin(), filtered_points.end(), map_frame_world->begin());
                        },
                        "EROAM: map_frame_world copy");
                }
                else
                {
                    LOG(WARNING) << "EROAM: world frame point vector is empty for keyframe, skipping.";
                }
            }

            // accumulate the whole event sphere
            global_map_event_sphere_vec_.emplace_back(event_sphere_frame_world);

            eroam::common::Timer::Evaluate(
                [&]()
                {
                    // // publish local_map_event_sphere_
                    // sensor_msgs::PointCloud2 local_map_event_sphere_msg;
                    // pcl::toROSMsg(*local_map_event_sphere_, local_map_event_sphere_msg);
                    // local_map_event_sphere_msg.header.frame_id = "world";
                    // local_map_event_sphere_pub_.publish(local_map_event_sphere_msg);
                    if (iskeyframe)
                    {
                        pcl::transformPointCloud(*map_frame_world, *map_frame_world, vis_transform.matrix());
                        sensor_msgs::PointCloud2 map_frame_msg2;
                        pcl::toROSMsg(*map_frame_world, map_frame_msg2);
                        map_frame_msg2.header.frame_id = "world";
                        map_frame_pub_.publish(map_frame_msg2);
                    }

                    // publish event point cloud
                    sensor_msgs::PointCloud2 ev_pc_msg2;
                    pcl::toROSMsg(*event_sphere_frame_world_vis, ev_pc_msg2);
                    ev_pc_msg2.header.frame_id = "world";
                    ev_sphere_frame_pub_.publish(ev_pc_msg2);

                    // publish camera frame
                    CloudPtr camera_frame = GenerateCameraFrame(pose_se3);
                    Eigen::Isometry3d vis_camera_transform = Eigen::Isometry3d::Identity();
                    vis_camera_transform.rotate(Eigen::AngleAxisd(3 * M_PI / 2, Eigen::Vector3d::UnitX()));
                    CloudPtr camera_frame_vis(new PointCloudType);
                    pcl::transformPointCloud(*camera_frame, *camera_frame_vis, vis_camera_transform.matrix());

                    sensor_msgs::PointCloud2 camera_frame_msg;
                    pcl::toROSMsg(*camera_frame_vis, camera_frame_msg);
                    camera_frame_msg.header.frame_id = "world";
                    camera_frame_pub_.publish(camera_frame_msg);
                },
                "EROAM: pubish everything");
        }

    private:
        /**
         * @brief Aligns a new event frame with the existing map using the Event Spherical ICP (ES-ICP) algorithm.
         * This is the core of the pose estimation pipeline, implementing the robust point-to-line
         * geometric optimization detailed in Section III-D of the paper.
         * @param ev_sphere_frame The current motion-compensated event spherical frame to be aligned.
         * @return The estimated SE(3) pose of the current frame, representing the camera's rotation.
         */
        SE3 AlignWithLocalSphere(CloudPtr ev_sphere_frame)
        {
            SO3 pose;
            if (estimated_poses_se3_.size() >= 2)
            {
                // the initial guess is the relative motion between the last two frames
                // T_i-2_i-1 = T_i-2_w * T_w_i-1
                // T_i-1_i = T_i-1_w * T_w_i
                // T_w_i = T_w_i-1 * (T_i-2_w * T_w_i-1)
                SE3 T1 = estimated_poses_se3_[estimated_poses_se3_.size() - 1];
                SE3 T2 = estimated_poses_se3_[estimated_poses_se3_.size() - 2];
                pose = (T1 * (T2.inverse() * T1)).so3();
            }

            int edge_size = ev_sphere_frame->size();

            // point to edge error, 3D
            std::vector<bool> effect_edge(edge_size, false);
            std::vector<Eigen::Matrix<double, 3, 3>> jacob_edge(edge_size); // error dimension is 3
            std::vector<Vec3d> errors_edge(edge_size);

            std::vector<int> index_edge(edge_size);
            std::for_each(index_edge.begin(), index_edge.end(), [cnt = 0](int &i) mutable
                          { i = cnt++; });

            /// gauss-newton optimization, calculate jacobian matrix, hesse matrix and error
            /// form the equation H dx = g, where H is the hesse matrix, dx is the delta of pose, g is the error
            /// find the optimal dx, and update the pose
            /// H = sum(J^T * J), g = sum(-J^T * error)
            int iter = 0;
            for (; iter < options_.max_iteration_; ++iter)
            {
                // edge points
                std::for_each(std::execution::par_unseq, index_edge.begin(), index_edge.end(), [&](int idx)
                              {
                    // edge query point
                    Vec3d q = ToVec3d(ev_sphere_frame->points[idx]);
                    Vec3d qs = pose * q;

                    PointVector nn_points;     // Stores nearest points
                    vector<float> nn_dists_sq; // Stores squared distances
                    int k_neighbors = 5;       // Number of neighbors to find
                    ikdtree_sphere_->Nearest_Search(ToPointType(qs), k_neighbors, nn_points, nn_dists_sq);

                    effect_edge[idx] = false;

                    if (nn_points.size() >= 3) {
                        std::vector<Vec3d> nn_eigen;
                        for (const auto &p : nn_points) { // Iterate through found points
                            nn_eigen.emplace_back(ToVec3d(p));
                        }
                        // point to point residual
                        Vec3d d, p0;
                        if (!math::FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
                            return;
                        }

                        // point to line residual
                        // err = d x (qs - p0), where x is cross product, so err is a 3D vector
                        Vec3d err = SO3::hat(d) * (qs - p0);
                        if (err.norm() > options_.max_line_distance_) {
                            return;
                        }

                        effect_edge[idx] = true;

                        // build residual
                        // error = d x (Rq + T - p0)
                        // J = [-d^Rq^, d^]
                        Eigen::Matrix<double, 3, 3> J;      // J is 3x3
                        J.block<3, 3>(0, 0) = -SO3::hat(d) * pose.matrix() * SO3::hat(q);
                        jacob_edge[idx] = J;
                        errors_edge[idx] = err;
                    } });

                // gauss-newton optimization
                double total_res = 0;
                int effective_num = 0;

                Mat3d H = Mat3d::Zero(); // hesse matrix, 6x6
                Vec3d g = Vec3d::Zero(); // error, 6x1

                for (const auto &idx : index_edge)
                {
                    if (effect_edge[idx])
                    {
                        H += jacob_edge[idx].transpose() * jacob_edge[idx];
                        g += -jacob_edge[idx].transpose() * errors_edge[idx];
                        effective_num++;
                        total_res += errors_edge[idx].norm();
                    }
                }

                if (effective_num < options_.min_effective_pts_)
                {
                    LOG(WARNING) << "effective num too small: " << effective_num;

                    SE3 pose_se3;
                    pose_se3.so3() = pose;
                    return pose_se3;
                }

                // solve dx, dx = H^-1 * g, the dx is optimal update of pose
                Vec3d dx = H.inverse() * g;
                pose = pose * SO3::exp(dx.head<3>());

                if (dx.norm() < options_.eps_)
                {
                    LOG(INFO) << "converged, dx = " << dx.transpose();
                    break;
                }
            }

            LOG(INFO) << "AlignWithLocalSphere: iter num: " << iter;

            SE3 pose_se3;
            pose_se3.so3() = pose;
            return pose_se3;
        }

        /**
         * @brief Determines whether the current frame should be designated as a new keyframe.
         * As described in Section III-E, a new keyframe is created if the rotational change
         * since the last keyframe exceeds a predefined angle, ensuring the map is updated
         * only when significant new information is available.
         * @param current_pose The latest estimated pose of the camera.
         * @return True if the frame should be a keyframe, false otherwise.
         */
        bool IsKeyframe(const SE3 &current_pose)
        {
            // the current frame id is far away from the last keyframe id
            if ((cnt_frame_ - last_kf_id_) > 1000)
            {
                return true;
            }

            // the relative motion between the current frame and the last keyframe is large enough
            // T_delta = T_i-1_w * T_w_i
            SE3 delta = last_kf_pose_.inverse() * current_pose;
            return delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
        }

        void LocalMapUpdating(SE3 &pose, CloudPtr edge_world, CloudPtr surf_world, CloudPtr scan_world);

        // --- Member Variables ---
        Options options_;
        DVSIntrinsics dvs_intrinsics_;
        ros::Subscriber ev_sub_; // ROS subscriber for event messages.
        std::string pose_result_file_;
        std::string panoramic_output_file_;
        // A lookup table to quickly retrieve the undistorted 3D point on the sphere for any given pixel coordinate (u,v).
        // This pre-computation significantly speeds up the event projection process.
        std::vector<std::vector<Eigen::Vector3d>> precomputed_undistortion_vec_;

        int cnt_frame_ = 0;                       // A counter for the number of processed frames.
        int last_kf_id_ = 0;                      // The id of the last registered keyframe.
        double dvxplorer_correction_ratio_ = 1.0; // A correction ratio for DVXplorer cameras under high event rates.

        CloudPtr local_map_event_sphere_ = nullptr; // The local event sphere (used for initialization).
        // A vector of all transformed event spherical frames. This is used for generating the final high-resolution panoramic image.
        std::vector<CloudPtr> global_map_event_sphere_vec_;

        // History of estimated poses (SE3) and their corresponding timestamps, used for motion prediction and trajectory recording.
        std::vector<SE3> estimated_poses_se3_;
        std::vector<ros::Time> estimated_poses_ts_;

        SE3 last_kf_pose_;                               // Stores the pose of the last registered keyframe.
        std::deque<CloudPtr> local_event_sphere_frames_; // Stores local event frames (currently unused in this simplified version).
        std::mutex local_map_updating_mutex_;            // A mutex for thread-safe map updates.

        // The incremental k-d tree used to store and efficiently query the global spherical map points.
        KD_TREE<PointType>::Ptr ikdtree_sphere_ = nullptr;

        bool save_frames_ = false;

        ros::NodeHandle nh_;
        ros::Publisher ev_sphere_frame_pub_;
        ros::Publisher map_frame_pub_;
        ros::Publisher local_map_event_sphere_pub_;
        ros::Publisher edge_pc_pub_;
        ros::Publisher surf_pc_pub_;
        ros::Publisher pose_pub_;
        ros::Publisher path_pub_;
        ros::Publisher camera_frame_pub_;

        // panorama generation params
        int panoramic_height_res_ = 1000;
        double panoramic_top_brightness_percent_ = 0.1;

        double panoramic_longitude_start_ = -180.0;
        double panoramic_longitude_end_ = 180.0;
        double panoramic_latitude_start_ = -30.0;
        double panoramic_latitude_end_ = 30.0;
    };

} // namespace eroam

#endif // EROAM_HPP