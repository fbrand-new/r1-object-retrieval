/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "checkObjectGrasped.h"

YARP_LOG_COMPONENT(CHECK_OBJECT_GRASPED, "r1_obr.checkObjectGrasped")

/****************************************************************/
CheckObjectGrasped::CheckObjectGrasped() : 
    m_period(1.0),  // 1Hz as requested
    m_consecutive_frames_threshold(5),
    m_detection_timeout(2.0),
    m_observation_count(5),
    m_success_threshold(0.6),  // 60% success rate
    m_current_state(CheckingState::UNKNOWN),
    m_transform_valid(false),
    m_positive_detections(0),
    m_total_observations(0)
{
    m_detection_port_name = "/checkObjectGrasped/detections:i";
    m_rpc_port_name = "/checkObjectGrasped/rpc";
    m_status_port_name = "/checkObjectGrasped/status:o";
    m_gaze_port_name = "/checkObjectGrasped/gaze:o";
    m_robot_name = "cer";
    m_target_object = "bottle";
    m_hand_frame = "r_hand";
    m_base_frame = "mobile_base_body_link";
}

/****************************************************************/
bool CheckObjectGrasped::configure(ResourceFinder &rf)
{
    // ------------ Generic config ------------ //
    if(rf.check("period")) {
        m_period = rf.find("period").asFloat32();
    }
    
    if(rf.check("robot")) {
        m_robot_name = rf.find("robot").asString();
    }
    
    if(rf.check("target_object")) {
        m_target_object = rf.find("target_object").asString();
    }
    
    if(rf.check("hand_frame")) {
        m_hand_frame = rf.find("hand_frame").asString();
    }
    
    if(rf.check("base_frame")) {
        m_base_frame = rf.find("base_frame").asString();
    }
    
    if(rf.check("consecutive_frames_threshold")) {
        m_consecutive_frames_threshold = rf.find("consecutive_frames_threshold").asInt32();
    }
    
    if(rf.check("detection_timeout")) {
        m_detection_timeout = rf.find("detection_timeout").asFloat64();
    }
    
    if(rf.check("observation_count")) {
        m_observation_count = rf.find("observation_count").asInt32();
    }
    
    if(rf.check("success_threshold")) {
        m_success_threshold = rf.find("success_threshold").asFloat64();
    }
    
    if(rf.check("detection_port")) {
        m_detection_port_name = rf.find("detection_port").asString();
    }
    
    if(rf.check("rpc_port")) {
        m_rpc_port_name = rf.find("rpc_port").asString();
    }
    
    if(rf.check("status_port")) {
        m_status_port_name = rf.find("status_port").asString();
    }
    
    if(rf.check("gaze_port")) {
        m_gaze_port_name = rf.find("gaze_port").asString();
    }

    // ------------ ROS2 TF2 setup ------------ //
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    m_ros_node = std::make_shared<rclcpp::Node>("check_object_grasped_node");
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(m_ros_node->get_clock());
    m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

    // Create timer for periodic TF lookup
    m_ros_timer = m_ros_node->create_wall_timer(
        std::chrono::duration<double>(m_period), 
        std::bind(&CheckObjectGrasped::tfCallback, this)
    );

    // Start ROS2 spinner in separate thread
    m_ros_spinner = std::make_unique<std::thread>([this]() {
        rclcpp::spin(m_ros_node);
    });
    
    yCInfo(CHECK_OBJECT_GRASPED) << "ROS2 TF2 client configured successfully";

    // ------------ Port setup ------------ //
    m_detection_port.useCallback(*this);
    if(!m_detection_port.open(m_detection_port_name)) {
        yCError(CHECK_OBJECT_GRASPED) << "Cannot open detection port" << m_detection_port_name;
        return false;
    }
    yCInfo(CHECK_OBJECT_GRASPED) << "Opened detection port" << m_detection_port_name;
    
    if(!m_rpc_port.open(m_rpc_port_name)) {
        yCError(CHECK_OBJECT_GRASPED) << "Cannot open RPC port" << m_rpc_port_name;
        return false;
    }
    m_rpc_port.setReader(*this);
    yCInfo(CHECK_OBJECT_GRASPED) << "Opened RPC port" << m_rpc_port_name;
    
    if(!m_status_port.open(m_status_port_name)) {
        yCError(CHECK_OBJECT_GRASPED) << "Cannot open status port" << m_status_port_name;
        return false;
    }
    yCInfo(CHECK_OBJECT_GRASPED) << "Opened status port" << m_status_port_name;
    
    if(!m_gaze_port.open(m_gaze_port_name)) {
        yCError(CHECK_OBJECT_GRASPED) << "Cannot open gaze port" << m_gaze_port_name;
        return false;
    }
    yCInfo(CHECK_OBJECT_GRASPED) << "Opened gaze port" << m_gaze_port_name;
    
    yCInfo(CHECK_OBJECT_GRASPED) << "Configuration completed. Target object:" << m_target_object;
    yCInfo(CHECK_OBJECT_GRASPED) << "Hand frame:" << m_hand_frame << ", Base frame:" << m_base_frame;
    
    return true;
}

/****************************************************************/
bool CheckObjectGrasped::close()
{
    // Stop ROS2 components
    if (m_ros_node) {
        rclcpp::shutdown();
    }
    
    if (m_ros_spinner && m_ros_spinner->joinable()) {
        m_ros_spinner->join();
    }
    
    if (!m_detection_port.isClosed()) {
        m_detection_port.close();
    }
    
    if (!m_rpc_port.isClosed()) {
        m_rpc_port.close();
    }
    
    if (!m_status_port.isClosed()) {
        m_status_port.close();
    }
    
    if (!m_gaze_port.isClosed()) {
        m_gaze_port.close();
    }
    
    return true;
}

/****************************************************************/
double CheckObjectGrasped::getPeriod()
{
    return m_period;
}

/****************************************************************/
bool CheckObjectGrasped::updateModule()
{
    if (isStopping()) {
        return false;
    }
    
    // State machine logic
    switch (m_current_state) {
        case CheckingState::UNKNOWN:
            // Just publish status, wait for commands
            break;
            
        case CheckingState::GAZE:
            // Send gaze command once, then transition to CHECKING
            if (sendGazeCommand()) {
                yCInfo(CHECK_OBJECT_GRASPED) << "Gaze command sent, starting observation collection";
            } else {
                yCWarning(CHECK_OBJECT_GRASPED) << "Failed to send gaze command, continuing anyway";
            }
            m_current_state = CheckingState::CHECKING;
            break;
            
        case CheckingState::CHECKING:
            // Just collect observations, don't send gaze commands
            
            // Check if we have enough observations
            if (m_total_observations >= m_observation_count) {
                double success_rate = static_cast<double>(m_positive_detections) / m_total_observations;
                
                if (success_rate >= m_success_threshold) {
                    m_current_state = CheckingState::SUCCESS;
                    yCInfo(CHECK_OBJECT_GRASPED) << "SUCCESS: Object detected in" << m_positive_detections 
                                                << "/" << m_total_observations << "observations";
                } else {
                    m_current_state = CheckingState::FAILED;
                    yCInfo(CHECK_OBJECT_GRASPED) << "FAILED: Object detected in only" << m_positive_detections 
                                                << "/" << m_total_observations << "observations";
                }
                resetDetectionHistory();
            }
            break;
            
        case CheckingState::SUCCESS:
        case CheckingState::FAILED:
            // Wait for new commands, just like UNKNOWN
            break;
    }
    
    // Always publish current status
    publishStatus();
    
    return true;
}

/****************************************************************/
bool CheckObjectGrasped::respond(const Bottle& command, Bottle& reply)
{
    string cmd = command.get(0).asString();
    
    // Accept commands only in UNKNOWN, SUCCESS, or FAILED states
    if (m_current_state == CheckingState::CHECKING || m_current_state == CheckingState::GAZE) {
        reply.addString("busy");
        yCWarning(CHECK_OBJECT_GRASPED) << "Module is currently checking, please wait";
        return true;
    }
    
    if (cmd == "check") {
        m_current_state = CheckingState::GAZE;  // Start with gaze command
        resetDetectionHistory();
        m_checking_start_time = Time::now();
        reply.addString("ok");
        yCInfo(CHECK_OBJECT_GRASPED) << "Started checking for object grasp";
        return true;
    }
    else if (cmd == "stop") {
        m_current_state = CheckingState::UNKNOWN;
        resetDetectionHistory();
        reply.addString("ok");
        yCInfo(CHECK_OBJECT_GRASPED) << "Stopped checking, state reset to UNKNOWN";
        return true;
    }
    else if (cmd == "status") {
        reply.addString(stateToString(m_current_state));
        return true;
    }
    else if (cmd == "set_target") {
        if (command.size() >= 2) {
            m_target_object = command.get(1).asString();
            resetDetectionHistory();
            reply.addString("ok");
            yCInfo(CHECK_OBJECT_GRASPED) << "Target object set to:" << m_target_object;
        } else {
            reply.addString("error: missing object name");
        }
        return true;
    }
    else if (cmd == "get_target") {
        reply.addString(m_target_object);
        return true;
    }
    else if (cmd == "help") {
        reply.addString("Available commands:");
        reply.addString("check - start checking for grasped object");
        reply.addString("stop - stop checking and reset to UNKNOWN");
        reply.addString("status - get current state (UNKNOWN/CHECKING/SUCCESS/FAILED)");
        reply.addString("set_target <object_name> - set target object to detect");
        reply.addString("get_target - get current target object name");
        reply.addString("help - show this help");
        return true;
    }
    
    reply.addString("unknown command");
    return false;
}

/****************************************************************/
void CheckObjectGrasped::onRead(Bottle& detections)
{
    // Only process detections when in CHECKING state (not GAZE)
    if (m_current_state != CheckingState::CHECKING) {
        return;
    }
    
    m_last_detection_time = Time::now();
    bool object_found = processDetections(detections);
    
    // Count this observation
    m_total_observations++;
    if (object_found) {
        m_positive_detections++;
    }
    
    yCDebug(CHECK_OBJECT_GRASPED) << "Observation" << m_total_observations << "/" << m_observation_count 
                                 << "- Object found:" << (object_found ? "YES" : "NO");
}

/****************************************************************/
bool CheckObjectGrasped::getHandTransform(Vector& position, Vector& orientation)
{
    std::lock_guard<std::mutex> lock(m_transform_mutex);
    
    if (!m_transform_valid) {
        yCError(CHECK_OBJECT_GRASPED) << "No valid transform available";
        return false;
    }
    
    // Extract position from the stored transform
    position.resize(3);
    position[0] = m_last_transform.transform.translation.x;
    position[1] = m_last_transform.transform.translation.y;
    position[2] = m_last_transform.transform.translation.z;
    
    // Extract orientation (quaternion)
    orientation.resize(4);
    orientation[0] = m_last_transform.transform.rotation.x;
    orientation[1] = m_last_transform.transform.rotation.y;
    orientation[2] = m_last_transform.transform.rotation.z;
    orientation[3] = m_last_transform.transform.rotation.w;
    
    return true;
}

/****************************************************************/
bool CheckObjectGrasped::processDetections(const Bottle& detections)
{
    bool object_found = false;
    
    // Parse detection data - assuming format: [object_name, confidence, x, y, w, h, ...]
    for (int i = 0; i < detections.size(); i += 6) {
        if (i + 1 < detections.size()) {
            string detected_object = detections.get(i).asString();
            double confidence = detections.get(i + 1).asFloat64();
            
            // Check if this is our target object with sufficient confidence
            if (detected_object == m_target_object && confidence > 0.5) {
                object_found = true;
                yCDebug(CHECK_OBJECT_GRASPED) << "Target object" << m_target_object 
                                             << "detected with confidence" << confidence;
                break;
            }
        }
    }
    
    return object_found;
}

/****************************************************************/
bool CheckObjectGrasped::isObjectConsistentlyDetected()
{
    if (m_detection_history.size() < static_cast<size_t>(m_consecutive_frames_threshold)) {
        return false;
    }
    
    // Check if all recent detections are positive
    for (bool detection : m_detection_history) {
        if (!detection) {
            return false;
        }
    }
    
    return true;
}

/****************************************************************/
void CheckObjectGrasped::resetDetectionHistory()
{
    m_detection_history.clear();
    m_positive_detections = 0;
    m_total_observations = 0;
}

/****************************************************************/
bool CheckObjectGrasped::sendGazeCommand()
{
    Vector hand_pos(3), hand_orient(4);
    if (!getHandTransform(hand_pos, hand_orient)) {
        yCWarning(CHECK_OBJECT_GRASPED) << "Cannot get hand transform for gaze command";
        return false;
    }
    
    // Prepare gaze command - assuming format: [x, y, z]
    Bottle& cmd = m_gaze_port.prepare();
    cmd.clear();
    cmd.addString("look_at_point");
    cmd.addFloat64(hand_pos[0]);
    cmd.addFloat64(hand_pos[1]);
    cmd.addFloat64(hand_pos[2]);
    
    m_gaze_port.write();
    
    yCDebug(CHECK_OBJECT_GRASPED) << "Sent gaze command: look at point [" 
                                 << hand_pos[0] << ", " << hand_pos[1] << ", " << hand_pos[2] << "]";
    return true;
}

/****************************************************************/
void CheckObjectGrasped::publishStatus()
{
    Bottle& status = m_status_port.prepare();
    status.clear();
    
        // For external consumers, both GAZE and CHECKING appear as "CHECKING"
    status.addString(stateToString(m_current_state));
    
    // Add additional info for CHECKING state
    if (m_current_state == CheckingState::CHECKING) {
        status.addInt32(m_total_observations);
        status.addInt32(m_observation_count);
        status.addInt32(m_positive_detections);
    }

    
    m_status_port.write();
}

/****************************************************************/
string CheckObjectGrasped::stateToString(CheckingState state)
{
    switch (state) {
        case CheckingState::UNKNOWN: return "UNKNOWN";
        case CheckingState::GAZE: return "CHECKING";     // External view: appears as CHECKING
        case CheckingState::CHECKING: return "CHECKING";
        case CheckingState::SUCCESS: return "SUCCESS";
        case CheckingState::FAILED: return "FAILED";
        default: return "INVALID";
    }
}

/****************************************************************/
void CheckObjectGrasped::tfCallback()
{
    try {
        // Try to get the latest transform
        geometry_msgs::msg::TransformStamped transform_stamped = 
            m_tf_buffer->lookupTransform(m_base_frame, m_hand_frame, tf2::TimePointZero);
        
        // Store the transform in a thread-safe way
        {
            std::lock_guard<std::mutex> lock(m_transform_mutex);
            m_last_transform = transform_stamped;
            m_transform_valid = true;
        }
        
        yCDebug(CHECK_OBJECT_GRASPED) << "Transform updated successfully";
        
    } catch (const tf2::TransformException & ex) {
        yCWarning(CHECK_OBJECT_GRASPED) << "Could not get transform from" << m_hand_frame 
                                       << "to" << m_base_frame << ":" << ex.what();
        
        std::lock_guard<std::mutex> lock(m_transform_mutex);
        m_transform_valid = false;
    }
}
