/*
Copyright (c) 2022, Marvelmind Robotics
All rights reserved.
Modified for ROS 2

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <fcntl.h>
#include <string.h>
#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "marvelmind_driver/srv/marvelmind_api.hpp"

// Include C header - adjust path based on your structure
extern "C" 
{
#include "marvelmind_driver/marvelmind_api.h"
}

#define ROS_NODE_NAME "marvelmind_api_ros"
#define MM_API_SEMAPHORE "/mm_api_semaphore"

using namespace std::chrono_literals;

///////////////////////////
// Global variables
///////////////////////////

static uint32_t mm_api_version = 0;
static char ttyFileName[256];
static bool openPortTryNeeded = true;
static sem_t *sem;
struct timespec ts;

///////////////////////////
// Helper functions
///////////////////////////

void semCallback()
{
    sem_post(sem);
}

void tryOpenPort(rclcpp::Logger logger) 
{
    if (ttyFileName[0] == 0) {
        if (mmOpenPort()) {
            RCLCPP_INFO(logger, "Port opened");
            openPortTryNeeded = false;
        }
        else {
            RCLCPP_INFO(logger, "Failed to open port. Retrying...");
        }
    } else {
        char portName[256];
        strcpy(portName, ttyFileName);
        if (mmOpenPortByName(portName)) {
            RCLCPP_INFO(logger, "Port opened: %s", portName);
            openPortTryNeeded = false;
        }
        else {
            RCLCPP_INFO(logger, "Failed to open port: %s. Retrying...", portName);
        }
    }
}

static void apiPrepare(int argc, char **argv, rclcpp::Logger logger)
{
    // Get port name from command line arguments (if specified)
    ttyFileName[0] = 0;
    if (argc >= 2) {
        strcpy(ttyFileName, argv[1]);
    }
    
    mm_api_version = 0;
    openPortTryNeeded = true;
    
    RCLCPP_INFO(logger, "Opening Marvelmind API");
    marvelmindAPILoad(NULL);
    
    if (mmAPIVersion(&mm_api_version)) {
        RCLCPP_INFO(logger, "Marvelmind API version: %d", (int)mm_api_version);
        tryOpenPort(logger);
    }
    else {
        RCLCPP_INFO(logger, "Failed to get Marvelmind API version");
    }
}

///////////////////////////
// ROS 2 Node Class
///////////////////////////

class MarvelmindAPINode : public rclcpp::Node
{
public:
    MarvelmindAPINode(int argc, char **argv) 
        : Node(ROS_NODE_NAME)
    {
        // Initialize semaphore
        sem = sem_open(MM_API_SEMAPHORE, O_CREAT, 0777, 0);
        
        // Prepare API
        apiPrepare(argc, argv, this->get_logger());
        
        // Create service
        service_ = this->create_service<marvelmind_driver::srv::MarvelmindAPI>(
            "marvelmind_api",
            std::bind(&MarvelmindAPINode::handleService, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Create timer (1 second interval)
        timer_ = this->create_wall_timer(
            1s, std::bind(&MarvelmindAPINode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Marvelmind API ROS 2 node started");
    }
    
    ~MarvelmindAPINode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Marvelmind API node");
        
        // Close port if opened
        mmClosePort();
        
        // Free Marvelmind API library memory
        marvelmindAPIFree();
        
        // Close semaphore
        if (sem != SEM_FAILED) {
            sem_close(sem);
        }
    }

private:
    /**
     * Timer callback - runs every second
     * Tries to open port and get device list
     */
    void timerCallback()
    {
        // Try to open port if not yet opened
        if (openPortTryNeeded) {
            tryOpenPort(this->get_logger());
        }
        
        // Skip if port still not open
        if (openPortTryNeeded)
            return;
        
        // Get devices list
        MarvelmindDevicesList md;
        mmGetDevicesList(&md);
    }
    
    /**
     * Service handler for Marvelmind API calls
     */
    void handleService(
        const std::shared_ptr<marvelmind_driver::srv::MarvelmindAPI::Request> request,
        std::shared_ptr<marvelmind_driver::srv::MarvelmindAPI::Response> response)
    {
        // Extract request data
        const std::vector<uint8_t>& req_vec = request->request;
        uint8_t* rq_data = const_cast<uint8_t*>(req_vec.data());
        uint32_t rq_size = static_cast<uint32_t>(req_vec.size());
        int64_t rq_command = request->command_id;

        // Prepare response buffer
        uint8_t response_buf[1024];
        uint32_t response_size = 1024;
        int32_t error_code = -1;

        // Call Marvelmind API
        bool result = marvelmindAPICall(
            rq_command, 
            rq_data, 
            rq_size, 
            response_buf, 
            &response_size, 
            &error_code
        );
        
        // Set response
        response->success = result;
        response->error_code = error_code;
        
        if (response->success) {
            // Copy response data
            response->response.resize(response_size);
            for (uint32_t i = 0; i < response_size; i++) {
                response->response[i] = response_buf[i];
            }
        }
        else {
            // Empty response on failure
            response->response.clear();
        }
        
        // Handle port-related commands
        switch(rq_command) {
            case MM_API_ID_OPEN_PORT:
            case MM_API_ID_OPEN_PORT_BY_NAME:
            case MM_API_ID_OPEN_PORT_UDP:
            case MM_API_ID_CLOSE_PORT:
            {
                if (openPortTryNeeded) {
                    RCLCPP_INFO(this->get_logger(), 
                               "Port access function call: automatic port open cancelled");
                    openPortTryNeeded = false;
                }
                break;
            }
            default:
                break;
        }
    }
    
    // Member variables
    rclcpp::Service<marvelmind_driver::srv::MarvelmindAPI>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

///////////////////////////
// Main function
///////////////////////////

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    try {
        // Create and spin node
        auto node = std::make_shared<MarvelmindAPINode>(argc, argv);
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger(ROS_NODE_NAME), 
                     "Exception in main: %s", e.what());
    }
    
    // Shutdown
    rclcpp::shutdown();
    
    return 0;
}
