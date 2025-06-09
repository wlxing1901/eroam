#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dv_ros_msgs/EventArray.h>
#include <algorithm>
#include <vector>
#include <set>

// Define parameters
std::string input_bag_path;
std::string output_bag_path;
std::string event_topic;
double segment_duration;
double start_time_offset;  // Added: start time offset (seconds)
double duration_limit;     // Added: processing duration limit (seconds)
std::vector<std::vector<int>> bad_pixels;

// Helper struct to store bad pixels
struct PixelCoord {
    int x, y;
    bool operator<(const PixelCoord& other) const {
        return (x < other.x) || (x == other.x && y < other.y);
    }
};

// Check if an event is a bad pixel
bool isBadPixel(const std::set<PixelCoord>& bad_pixel_set, int x, int y) {
    return bad_pixel_set.find(PixelCoord{x, y}) != bad_pixel_set.end();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "event_repackager");
    ros::NodeHandle nh("~");

    // Get parameters from the launch file
    nh.getParam("input_bag", input_bag_path);
    nh.getParam("output_bag", output_bag_path);
    nh.getParam("event_topic", event_topic);
    nh.getParam("segment_duration", segment_duration);
    
    // Added parameters, set default values
    nh.param("start_time_offset", start_time_offset, 0.0); // Default: start from the beginning
    nh.param("duration_limit", duration_limit, -1.0);      // Default: -1 means process until the end
    
    // Get the list of bad pixels
    XmlRpc::XmlRpcValue bad_pixels_list;
    nh.getParam("bad_pixels", bad_pixels_list);
    
    // Convert bad pixels to a set for fast lookup
    std::set<PixelCoord> bad_pixel_set;
    if (bad_pixels_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < bad_pixels_list.size(); ++i) {
            if (bad_pixels_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray &&
                bad_pixels_list[i].size() == 2) {
                int x = static_cast<int>(bad_pixels_list[i][0]);
                int y = static_cast<int>(bad_pixels_list[i][1]);
                bad_pixel_set.insert(PixelCoord{x, y});
            }
        }
    }

    // Output parameter information
    ROS_INFO("Processing parameters:");
    ROS_INFO("  Start time offset: %.3f seconds", start_time_offset);
    if (duration_limit > 0) {
        ROS_INFO("  Duration limit: %.3f seconds", duration_limit);
    } else {
        ROS_INFO("  Duration limit: process until end");
    }
    ROS_INFO("Bad pixels:");
    for (const auto& pixel : bad_pixel_set) {
        ROS_INFO("  (%d, %d)", pixel.x, pixel.y);
    }

    rosbag::Bag input_bag;
    rosbag::Bag output_bag;

    try {
        input_bag.open(input_bag_path, rosbag::bagmode::Read);
        output_bag.open(output_bag_path, rosbag::bagmode::Write);

        rosbag::View view(input_bag);

        ros::Time bag_start_time;
        ros::Time actual_start_time;  // Actual processing start time
        ros::Time actual_end_time;    // Actual processing end time
        ros::Time segment_start_time;
        std::vector<dv_ros_msgs::Event> current_events;

        bool is_first_event = true;
        bool processing_started = false;

        for (const rosbag::MessageInstance& m : view) {
            if (m.getTopic() == event_topic) {
                auto event_array = m.instantiate<dv_ros_msgs::EventArray>();
                if (event_array != nullptr) {
                    if (is_first_event) {
                        bag_start_time = event_array->events.front().ts;
                        actual_start_time = bag_start_time + ros::Duration(start_time_offset);
                        
                        if (duration_limit > 0) {
                            actual_end_time = actual_start_time + ros::Duration(duration_limit);
                        }
                        
                        is_first_event = false;
                        ROS_INFO("Bag start time: %.6f", bag_start_time.toSec());
                        ROS_INFO("Processing start time: %.6f", actual_start_time.toSec());
                        if (duration_limit > 0) {
                            ROS_INFO("Processing end time: %.6f", actual_end_time.toSec());
                        }
                    }

                    for (const auto& event : event_array->events) {
                        // Check if the start time has been reached
                        if (event.ts < actual_start_time) {
                            continue;  // Skip events before the start time
                        }
                        
                        // Check if the end time has been exceeded
                        if (duration_limit > 0 && event.ts >= actual_end_time) {
                            goto finish_processing;  // Break out of the nested loop
                        }
                        
                        if (!processing_started) {
                            processing_started = true;
                            segment_start_time = actual_start_time;
                            ROS_INFO("Started processing events from time: %.6f", event.ts.toSec());
                        }
                        
                        // Check if it is a bad pixel, skip if so
                        if (isBadPixel(bad_pixel_set, event.x, event.y)) {
                            continue;
                        }

                        ros::Duration segment_elapsed = event.ts - segment_start_time;

                        if (segment_elapsed.toSec() < segment_duration) {
                            current_events.push_back(event);
                        } else {
                            // Create a new EventArray and write it to the output bag
                            if (!current_events.empty()) {
                                dv_ros_msgs::EventArray new_event_array;
                                new_event_array.header = event_array->header;
                                new_event_array.header.stamp = segment_start_time;
                                new_event_array.events = current_events;

                                output_bag.write(event_topic, segment_start_time, new_event_array);
                            }

                            // Reset the current event list and time
                            current_events.clear();
                            segment_start_time += ros::Duration(segment_duration);

                            while (event.ts - segment_start_time >= ros::Duration(segment_duration)) {
                                segment_start_time += ros::Duration(segment_duration);
                            }
                            current_events.push_back(event);
                        }
                    }
                }
            } else {
                // Optional: handle messages from other topics
                // Check if the message time is within the processing range
                if (processing_started && m.getTime() >= actual_start_time) {
                    if (duration_limit <= 0 || m.getTime() < actual_end_time) {
                        // output_bag.write(m.getTopic(), m.getTime(), m);
                    }
                }
            }
        }

        finish_processing:
        // Process the last group of events (if any)
        if (!current_events.empty()) {
            dv_ros_msgs::EventArray new_event_array;
            new_event_array.header.stamp = segment_start_time;
            new_event_array.events = current_events;
            output_bag.write(event_topic, segment_start_time, new_event_array);
        }

        input_bag.close();
        output_bag.close();
        
        if (processing_started) {
            ROS_INFO("Event repackaging completed successfully. Filtered out %zu bad pixels.", bad_pixel_set.size());
        } else {
            ROS_WARN("No events were processed. Check start_time_offset parameter.");
        }
        
    } catch (rosbag::BagException& e) {
        ROS_ERROR("Error processing bag file: %s", e.what());
        return 1;
    }

    return 0;
}