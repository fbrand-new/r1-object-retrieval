/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CHECK_OBJECT_GRASPED_H
#define CHECK_OBJECT_GRASPED_H

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <string>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

// State machine states
enum class CheckingState {
    UNKNOWN,
    GAZE,      // Internal state: send gaze command once
    CHECKING,  // Collecting observations
    SUCCESS,
    FAILED
};

class CheckObjectGrasped : public RFModule, public TypedReaderCallback<Bottle>
{
private:
    double                      m_period;
       
    // Object detection input port
    BufferedPort<Bottle>        m_detection_port;
    string                      m_detection_port_name;
    
    // RPC port for commands
    RpcServer                   m_rpc_port;
    string                      m_rpc_port_name;
    
    // Status output port
    BufferedPort<Bottle>        m_status_port;
    string                      m_status_port_name;
    
    // Gaze controller client port
    RpcClient                   m_gaze_port;
    string                      m_gaze_port_name;
    
    // Configuration parameters
    string                      m_robot_name;
    string                      m_target_object;
    string                      m_hand_frame;
    string                      m_base_frame;
    int                         m_consecutive_frames_threshold;
    double                      m_detection_timeout;
    int                         m_observation_count;
    double                      m_success_threshold;
    
    // State variables
    CheckingState               m_current_state;
    deque<bool>                 m_detection_history;
    Time                        m_last_detection_time;
    Time                        m_checking_start_time;
    int                         m_positive_detections;
    int                         m_total_observations;
    
    // ROS2 TF2 components
    rclcpp::Node::SharedPtr     m_ros_node;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::unique_ptr<std::thread> m_ros_spinner;
    rclcpp::TimerBase::SharedPtr m_ros_timer;
    
    // Thread-safe transform storage
    geometry_msgs::msg::TransformStamped m_last_transform;
    std::mutex m_transform_mutex;
    bool m_transform_valid;
    
    // Helper methods
    bool getHandTransform(Vector& position, Vector& orientation);
    bool processDetections(const Bottle& detections);
    bool sendGazeCommand();
    void publishStatus();
    void resetDetectionHistory();
    void tfCallback();
    string stateToString(CheckingState state);


public:
    CheckObjectGrasped();
    virtual bool configure(ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool respond(const Bottle& command, Bottle& reply);

    using TypedReaderCallback<Bottle>::onRead;
    void onRead(Bottle& btl) override;
};

#endif
