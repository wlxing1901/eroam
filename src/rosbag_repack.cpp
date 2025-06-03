#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dv_ros_msgs/EventArray.h>
#include <algorithm>
#include <vector>
#include <set>

// 定义参数
std::string input_bag_path;
std::string output_bag_path;
std::string event_topic;
double segment_duration;
double start_time_offset;  // 新增：开始时间偏移（秒）
double duration_limit;     // 新增：处理持续时间限制（秒）
std::vector<std::vector<int>> bad_pixels;

// 辅助结构体用于存储坏点
struct PixelCoord {
    int x, y;
    bool operator<(const PixelCoord& other) const {
        return (x < other.x) || (x == other.x && y < other.y);
    }
};

// 检查事件是否为坏点
bool isBadPixel(const std::set<PixelCoord>& bad_pixel_set, int x, int y) {
    return bad_pixel_set.find(PixelCoord{x, y}) != bad_pixel_set.end();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "event_repackager");
    ros::NodeHandle nh("~");

    // 从launch文件获取参数
    nh.getParam("input_bag", input_bag_path);
    nh.getParam("output_bag", output_bag_path);
    nh.getParam("event_topic", event_topic);
    nh.getParam("segment_duration", segment_duration);
    
    // 新增参数，设置默认值
    nh.param("start_time_offset", start_time_offset, 0.0);  // 默认从开始
    nh.param("duration_limit", duration_limit, -1.0);      // 默认-1表示处理到结束
    
    // 获取坏点列表
    XmlRpc::XmlRpcValue bad_pixels_list;
    nh.getParam("bad_pixels", bad_pixels_list);
    
    // 将坏点转换为set以便快速查找
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

    // 输出参数信息
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
        ros::Time actual_start_time;  // 实际处理开始时间
        ros::Time actual_end_time;    // 实际处理结束时间
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
                        // 检查是否达到开始时间
                        if (event.ts < actual_start_time) {
                            continue;  // 跳过开始时间之前的事件
                        }
                        
                        // 检查是否超过结束时间
                        if (duration_limit > 0 && event.ts >= actual_end_time) {
                            goto finish_processing;  // 跳出双重循环
                        }
                        
                        if (!processing_started) {
                            processing_started = true;
                            segment_start_time = actual_start_time;
                            ROS_INFO("Started processing events from time: %.6f", event.ts.toSec());
                        }
                        
                        // 检查是否为坏点，如果是则跳过
                        if (isBadPixel(bad_pixel_set, event.x, event.y)) {
                            continue;
                        }

                        ros::Duration segment_elapsed = event.ts - segment_start_time;

                        if (segment_elapsed.toSec() < segment_duration) {
                            current_events.push_back(event);
                        } else {
                            // 创建新的EventArray并写入输出bag
                            if (!current_events.empty()) {
                                dv_ros_msgs::EventArray new_event_array;
                                new_event_array.header = event_array->header;
                                new_event_array.header.stamp = segment_start_time;
                                new_event_array.events = current_events;

                                output_bag.write(event_topic, segment_start_time, new_event_array);
                            }

                            // 重置当前事件列表和时间
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
                // 可选：处理其他topic的消息
                // 检查消息时间是否在处理范围内
                if (processing_started && m.getTime() >= actual_start_time) {
                    if (duration_limit <= 0 || m.getTime() < actual_end_time) {
                        // output_bag.write(m.getTopic(), m.getTime(), m);
                    }
                }
            }
        }

        finish_processing:
        // 处理最后一组事件（如果有的话）
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