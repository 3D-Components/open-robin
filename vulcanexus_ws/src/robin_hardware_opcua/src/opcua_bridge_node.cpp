/**
 * @file opcua_bridge_node.cpp
 * @brief High-performance OPC UA to ROS2 bridge using YAML configuration
 * 
 * Optimizations:
 * - Single poll timer per server (batch reads)
 * - Pre-parsed NodeIDs (parse once at startup)
 * - Minimal mutex contention
 * - Configurable via YAML file
 * - Automatic reconnection handling
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "robin_interfaces/srv/set_float32.hpp"

#include <yaml-cpp/yaml.h>

extern "C" {
#include <open62541/client.h>
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
}

using namespace std::chrono_literals;

class OpcUaServer;
class OpcUaBridgeNode;

/**
 * @brief Represents a single OPC UA server connection
 */
class OpcUaServer {
public:
    OpcUaServer(const std::string& name, const YAML::Node& config, OpcUaBridgeNode* bridge);
    ~OpcUaServer();

    bool connect();
    void disconnect();
    bool isConnected() const { return connected_; }
    void poll();  // Single poll function for all topics
    
    const std::string& getName() const { return name_; }

private:
    friend class OpcUaBridgeNode;
    
    static UA_NodeId parseNodeId(const std::string& node_id_str);
    bool readFloatUnlocked(const UA_NodeId& node_id, float& value);
    bool readBoolUnlocked(const UA_NodeId& node_id, bool& value);
    bool writeFloat(const UA_NodeId& node_id, float value);
    bool writeBool(const UA_NodeId& node_id, bool value);

    std::string name_;
    std::string url_;
    std::string auth_type_;
    std::string username_;
    std::string password_;

    UA_Client* client_{nullptr};
    std::atomic<bool> connected_{false};
    std::mutex client_mutex_;

    OpcUaBridgeNode* bridge_;

    // Topic configurations with pre-parsed NodeIDs
    struct TopicConfig {
        std::string name;
        UA_NodeId node_id;  // Pre-parsed!
        std::string type;
        rclcpp::PublisherBase::SharedPtr publisher;
    };
    std::vector<TopicConfig> topics_;

    // Service configurations with pre-parsed NodeIDs
    struct ServiceConfig {
        std::string name;
        UA_NodeId node_id;  // Pre-parsed!
        std::string type;
        rclcpp::ServiceBase::SharedPtr service;
    };
    std::vector<ServiceConfig> services_;
    
    rclcpp::TimerBase::SharedPtr poll_timer_;
};

/**
 * @brief Main ROS2 node that manages OPC UA server connections
 */
class OpcUaBridgeNode : public rclcpp::Node {
public:
    OpcUaBridgeNode() : Node("opcua_bridge") {
        RCLCPP_INFO(this->get_logger(), "OPC UA Bridge Node starting...");

        this->declare_parameter<std::string>("config_file", "");
        std::string config_file = this->get_parameter("config_file").as_string();
        
        if (config_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No config_file parameter provided!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

        if (!loadConfig(config_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config file");
            return;
        }

        // Reconnection timer (less frequent)
        reconnect_timer_ = this->create_wall_timer(
            5s, std::bind(&OpcUaBridgeNode::checkConnections, this));

        RCLCPP_INFO(this->get_logger(), "OPC UA Bridge initialized with %zu servers", 
            servers_.size());
    }

    ~OpcUaBridgeNode() {
        for (auto& server : servers_) {
            server->disconnect();
        }
    }

    template<typename T>
    typename rclcpp::Publisher<T>::SharedPtr createPublisher(const std::string& topic, size_t qos) {
        return this->create_publisher<T>(topic, qos);
    }

    rclcpp::TimerBase::SharedPtr createTimer(std::chrono::microseconds period, 
                                              std::function<void()> callback) {
        return this->create_wall_timer(period, callback);
    }

    template<typename ServiceT>
    typename rclcpp::Service<ServiceT>::SharedPtr createService(
        const std::string& name,
        std::function<void(const typename ServiceT::Request::SharedPtr,
                          typename ServiceT::Response::SharedPtr)> callback) {
        return this->create_service<ServiceT>(name, callback);
    }

    rclcpp::Logger getNodeLogger() { return this->get_logger(); }

private:
    bool loadConfig(const std::string& config_file) {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            
            if (!config["servers"]) {
                RCLCPP_ERROR(this->get_logger(), "No 'servers' section in config");
                return false;
            }

            for (const auto& server_entry : config["servers"]) {
                std::string server_name = server_entry.first.as<std::string>();
                YAML::Node server_config = server_entry.second;
                
                RCLCPP_INFO(this->get_logger(), "Configuring server: %s", server_name.c_str());
                
                auto server = std::make_unique<OpcUaServer>(server_name, server_config, this);
                servers_.push_back(std::move(server));
            }

            // Initial connection
            for (auto& server : servers_) {
                server->connect();
            }

            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Config error: %s", e.what());
            return false;
        }
    }

    void checkConnections() {
        for (auto& server : servers_) {
            if (!server->isConnected()) {
                RCLCPP_WARN(this->get_logger(), "Reconnecting to %s...", 
                    server->getName().c_str());
                server->connect();
            }
        }
    }

    std::vector<std::unique_ptr<OpcUaServer>> servers_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

// ============================================================================
// OpcUaServer Implementation
// ============================================================================

OpcUaServer::OpcUaServer(const std::string& name, const YAML::Node& config, OpcUaBridgeNode* bridge)
    : name_(name), bridge_(bridge) 
{
    url_ = config["url"].as<std::string>();

    if (config["auth"]) {
        auth_type_ = config["auth"]["type"].as<std::string>("anonymous");
        if (auth_type_ == "username_password") {
            username_ = config["auth"]["username"].as<std::string>("");
            password_ = config["auth"]["password"].as<std::string>("");
        }
    } else {
        auth_type_ = "anonymous";
    }

    RCLCPP_INFO(bridge_->getNodeLogger(), "[%s] URL: %s, Auth: %s", 
        name_.c_str(), url_.c_str(), auth_type_.c_str());

    // Create OPC UA client with optimized settings
    client_ = UA_Client_new();
    UA_ClientConfig* client_config = UA_Client_getConfig(client_);
    UA_ClientConfig_setDefault(client_config);
    client_config->timeout = 1000;  // 1 second timeout (fast fail)

    // Get poll rate from config (default 100 Hz)
    double poll_rate = config["poll_rate"] ? config["poll_rate"].as<double>() : 100.0;
    auto poll_period = std::chrono::microseconds(static_cast<int>(1000000.0 / poll_rate));

    // Parse topics - pre-parse NodeIDs
    if (config["topics"]) {
        for (const auto& topic_node : config["topics"]) {
            TopicConfig topic;
            topic.name = topic_node["name"].as<std::string>();
            std::string node_id_str = topic_node["node_id"].as<std::string>();
            topic.node_id = parseNodeId(node_id_str);
            topic.type = topic_node["type"].as<std::string>("float32");

            // Create publisher
            if (topic.type == "float32") {
                topic.publisher = bridge_->createPublisher<std_msgs::msg::Float32>(topic.name, 10);
            } else if (topic.type == "bool") {
                topic.publisher = bridge_->createPublisher<std_msgs::msg::Bool>(topic.name, 10);
            } else if (topic.type == "int32") {
                topic.publisher = bridge_->createPublisher<std_msgs::msg::Int32>(topic.name, 10);
            }

            RCLCPP_INFO(bridge_->getNodeLogger(), "[%s] Topic: %s (%s)",
                name_.c_str(), topic.name.c_str(), topic.type.c_str());

            topics_.push_back(std::move(topic));
        }
    }

    // Parse services - pre-parse NodeIDs
    if (config["services"]) {
        for (const auto& service_node : config["services"]) {
            ServiceConfig service;
            service.name = service_node["name"].as<std::string>();
            std::string node_id_str = service_node["node_id"].as<std::string>();
            service.node_id = parseNodeId(node_id_str);
            service.type = service_node["type"].as<std::string>("bool");

            // Store index for lambda capture (services_ not modified after this loop)
            size_t svc_idx = services_.size();

            if (service.type == "bool") {
                service.service = bridge_->createService<std_srvs::srv::SetBool>(
                    service.name,
                    [this, svc_idx](
                        const std_srvs::srv::SetBool::Request::SharedPtr request,
                        std_srvs::srv::SetBool::Response::SharedPtr response) {
                        if (this->writeBool(services_[svc_idx].node_id, request->data)) {
                            response->success = true;
                            response->message = request->data ? "ON" : "OFF";
                        } else {
                            response->success = false;
                            response->message = "Write failed - check connection";
                        }
                    });
            } else if (service.type == "float32") {
                service.service = bridge_->createService<robin_interfaces::srv::SetFloat32>(
                    service.name,
                    [this, svc_idx](
                        const robin_interfaces::srv::SetFloat32::Request::SharedPtr request,
                        robin_interfaces::srv::SetFloat32::Response::SharedPtr response) {
                        if (this->writeFloat(services_[svc_idx].node_id, request->data)) {
                            response->success = true;
                            response->message = "Set to " + std::to_string(request->data);
                        } else {
                            response->success = false;
                            response->message = "Write failed - check connection";
                        }
                    });
            }

            RCLCPP_INFO(bridge_->getNodeLogger(), "[%s] Service: %s (%s)",
                name_.c_str(), service.name.c_str(), service.type.c_str());

            services_.push_back(std::move(service));
        }
    }

    // Single poll timer for all topics (if any topics defined)
    if (!topics_.empty()) {
        poll_timer_ = bridge_->createTimer(poll_period, [this]() {
            this->poll();
        });
        RCLCPP_INFO(bridge_->getNodeLogger(), "[%s] Poll timer: %.1f Hz for %zu topics",
            name_.c_str(), poll_rate, topics_.size());
    }
}

OpcUaServer::~OpcUaServer() {
    disconnect();
    
    // Clean up pre-parsed NodeIDs
    for (auto& topic : topics_) {
        UA_NodeId_clear(&topic.node_id);
    }
    for (auto& service : services_) {
        UA_NodeId_clear(&service.node_id);
    }
    
    if (client_) {
        UA_Client_delete(client_);
    }
}

bool OpcUaServer::connect() {
    std::lock_guard<std::mutex> lock(client_mutex_);

    if (connected_) return true;

    RCLCPP_INFO(bridge_->getNodeLogger(), "[%s] Connecting to %s...", name_.c_str(), url_.c_str());

    UA_StatusCode status;
    if (auth_type_ == "username_password") {
        status = UA_Client_connectUsername(client_, url_.c_str(), 
            username_.c_str(), password_.c_str());
    } else {
        status = UA_Client_connect(client_, url_.c_str());
    }

    if (status != UA_STATUSCODE_GOOD) {
        RCLCPP_ERROR(bridge_->getNodeLogger(), "[%s] Connection failed: %s",
            name_.c_str(), UA_StatusCode_name(status));
        connected_ = false;
        return false;
    }

    connected_ = true;
    RCLCPP_INFO(bridge_->getNodeLogger(), "[%s] Connected!", name_.c_str());
    return true;
}

void OpcUaServer::disconnect() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (connected_) {
        UA_Client_disconnect(client_);
        connected_ = false;
    }
}

UA_NodeId OpcUaServer::parseNodeId(const std::string& node_id_str) {
    UA_NodeId node_id = UA_NODEID_NULL;
    
    size_t ns_pos = node_id_str.find("ns=");
    size_t s_pos = node_id_str.find(";s=");
    size_t i_pos = node_id_str.find(";i=");
    
    if (ns_pos != std::string::npos) {
        int ns = 0;
        if (s_pos != std::string::npos) {
            ns = std::stoi(node_id_str.substr(ns_pos + 3, s_pos - ns_pos - 3));
            std::string identifier = node_id_str.substr(s_pos + 3);
            node_id = UA_NODEID_STRING_ALLOC(ns, identifier.c_str());
        } else if (i_pos != std::string::npos) {
            ns = std::stoi(node_id_str.substr(ns_pos + 3, i_pos - ns_pos - 3));
            int identifier = std::stoi(node_id_str.substr(i_pos + 3));
            node_id = UA_NODEID_NUMERIC(ns, identifier);
        }
    }
    
    return node_id;
}

// Read without locking (caller must hold lock)
bool OpcUaServer::readFloatUnlocked(const UA_NodeId& node_id, float& value) {
    if (!connected_) return false;

    UA_Variant variant;
    UA_Variant_init(&variant);

    UA_StatusCode status = UA_Client_readValueAttribute(client_, node_id, &variant);

    if (status != UA_STATUSCODE_GOOD) {
        if (status == UA_STATUSCODE_BADCONNECTIONCLOSED ||
            status == UA_STATUSCODE_BADSECURECHANNELCLOSED ||
            status == UA_STATUSCODE_BADSERVERNOTCONNECTED) {
            connected_ = false;
        }
        return false;
    }

    bool success = false;
    if (UA_Variant_hasScalarType(&variant, &UA_TYPES[UA_TYPES_FLOAT])) {
        value = *(UA_Float*)variant.data;
        success = true;
    } else if (UA_Variant_hasScalarType(&variant, &UA_TYPES[UA_TYPES_DOUBLE])) {
        value = static_cast<float>(*(UA_Double*)variant.data);
        success = true;
    } else if (UA_Variant_hasScalarType(&variant, &UA_TYPES[UA_TYPES_INT32])) {
        value = static_cast<float>(*(UA_Int32*)variant.data);
        success = true;
    } else if (UA_Variant_hasScalarType(&variant, &UA_TYPES[UA_TYPES_INT16])) {
        value = static_cast<float>(*(UA_Int16*)variant.data);
        success = true;
    } else if (UA_Variant_hasScalarType(&variant, &UA_TYPES[UA_TYPES_UINT16])) {
        value = static_cast<float>(*(UA_UInt16*)variant.data);
        success = true;
    }

    UA_Variant_clear(&variant);
    return success;
}

bool OpcUaServer::readBoolUnlocked(const UA_NodeId& node_id, bool& value) {
    if (!connected_) return false;

    UA_Variant variant;
    UA_Variant_init(&variant);

    UA_StatusCode status = UA_Client_readValueAttribute(client_, node_id, &variant);

    if (status != UA_STATUSCODE_GOOD) {
        if (status == UA_STATUSCODE_BADCONNECTIONCLOSED ||
            status == UA_STATUSCODE_BADSECURECHANNELCLOSED ||
            status == UA_STATUSCODE_BADSERVERNOTCONNECTED) {
            connected_ = false;
        }
        return false;
    }

    bool success = false;
    if (UA_Variant_hasScalarType(&variant, &UA_TYPES[UA_TYPES_BOOLEAN])) {
        value = *(UA_Boolean*)variant.data;
        success = true;
    }

    UA_Variant_clear(&variant);
    return success;
}

bool OpcUaServer::writeBool(const UA_NodeId& node_id, bool value) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    
    if (!connected_) {
        RCLCPP_WARN(bridge_->getNodeLogger(), "[%s] Not connected", name_.c_str());
        return false;
    }

    UA_Variant variant;
    UA_Variant_init(&variant);
    UA_Boolean ua_value = value;
    UA_Variant_setScalar(&variant, &ua_value, &UA_TYPES[UA_TYPES_BOOLEAN]);

    UA_StatusCode status = UA_Client_writeValueAttribute(client_, node_id, &variant);

    if (status != UA_STATUSCODE_GOOD) {
        RCLCPP_ERROR(bridge_->getNodeLogger(), "[%s] Write failed: %s",
            name_.c_str(), UA_StatusCode_name(status));
        if (status == UA_STATUSCODE_BADCONNECTIONCLOSED ||
            status == UA_STATUSCODE_BADSECURECHANNELCLOSED ||
            status == UA_STATUSCODE_BADSERVERNOTCONNECTED) {
            connected_ = false;
        }
        return false;
    }

    return true;
}

bool OpcUaServer::writeFloat(const UA_NodeId& node_id, float value) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    
    if (!connected_) return false;

    UA_Variant variant;
    UA_Variant_init(&variant);
    UA_Float ua_value = value;
    UA_Variant_setScalar(&variant, &ua_value, &UA_TYPES[UA_TYPES_FLOAT]);

    UA_StatusCode status = UA_Client_writeValueAttribute(client_, node_id, &variant);

    if (status != UA_STATUSCODE_GOOD) {
        RCLCPP_ERROR(bridge_->getNodeLogger(), "[%s] Write failed: %s",
            name_.c_str(), UA_StatusCode_name(status));
        if (status == UA_STATUSCODE_BADCONNECTIONCLOSED ||
            status == UA_STATUSCODE_BADSECURECHANNELCLOSED ||
            status == UA_STATUSCODE_BADSERVERNOTCONNECTED) {
            connected_ = false;
        }
        return false;
    }

    return true;
}

// Single poll function - batch read all topics in one OPC UA request
void OpcUaServer::poll() {
    if (!connected_ || topics_.empty()) return;

    std::lock_guard<std::mutex> lock(client_mutex_);

    // Prepare batch read request
    size_t n = topics_.size();
    std::vector<UA_ReadValueId> read_ids(n);
    
    for (size_t i = 0; i < n; ++i) {
        UA_ReadValueId_init(&read_ids[i]);
        read_ids[i].nodeId = topics_[i].node_id;
        read_ids[i].attributeId = UA_ATTRIBUTEID_VALUE;
    }

    // Single batch read request
    UA_ReadRequest request;
    UA_ReadRequest_init(&request);
    request.nodesToRead = read_ids.data();
    request.nodesToReadSize = n;

    UA_ReadResponse response = UA_Client_Service_read(client_, request);

    if (response.responseHeader.serviceResult != UA_STATUSCODE_GOOD) {
        if (response.responseHeader.serviceResult == UA_STATUSCODE_BADCONNECTIONCLOSED ||
            response.responseHeader.serviceResult == UA_STATUSCODE_BADSECURECHANNELCLOSED ||
            response.responseHeader.serviceResult == UA_STATUSCODE_BADSERVERNOTCONNECTED) {
            connected_ = false;
        }
        UA_ReadResponse_clear(&response);
        return;
    }

    // Process results and publish
    for (size_t i = 0; i < response.resultsSize && i < n; ++i) {
        if (response.results[i].status != UA_STATUSCODE_GOOD) continue;
        
        UA_Variant* variant = &response.results[i].value;
        auto& topic = topics_[i];

        if (topic.type == "float32") {
            float value = 0.0f;
            bool ok = false;
            
            if (UA_Variant_hasScalarType(variant, &UA_TYPES[UA_TYPES_FLOAT])) {
                value = *(UA_Float*)variant->data;
                ok = true;
            } else if (UA_Variant_hasScalarType(variant, &UA_TYPES[UA_TYPES_DOUBLE])) {
                value = static_cast<float>(*(UA_Double*)variant->data);
                ok = true;
            } else if (UA_Variant_hasScalarType(variant, &UA_TYPES[UA_TYPES_INT32])) {
                value = static_cast<float>(*(UA_Int32*)variant->data);
                ok = true;
            } else if (UA_Variant_hasScalarType(variant, &UA_TYPES[UA_TYPES_INT16])) {
                value = static_cast<float>(*(UA_Int16*)variant->data);
                ok = true;
            } else if (UA_Variant_hasScalarType(variant, &UA_TYPES[UA_TYPES_UINT16])) {
                value = static_cast<float>(*(UA_UInt16*)variant->data);
                ok = true;
            }

            if (ok) {
                auto msg = std_msgs::msg::Float32();
                msg.data = value;
                auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32>>(topic.publisher);
                if (pub) pub->publish(msg);
            }
        } else if (topic.type == "bool") {
            if (UA_Variant_hasScalarType(variant, &UA_TYPES[UA_TYPES_BOOLEAN])) {
                auto msg = std_msgs::msg::Bool();
                msg.data = *(UA_Boolean*)variant->data;
                auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(topic.publisher);
                if (pub) pub->publish(msg);
            }
        }
    }

    UA_ReadResponse_clear(&response);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpcUaBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
