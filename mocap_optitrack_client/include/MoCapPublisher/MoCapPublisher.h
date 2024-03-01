#ifndef MOCAPPUBLISHER_H
#define MOCAPPUBLISHER_H

#include <vector>
#include <NatNetTypes.h>

#include "rclcpp/rclcpp.hpp"
#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


using namespace std;
using namespace std::chrono;
class MoCapPublisher: public rclcpp::Node
{
private:
    //Attributes
    rclcpp::Publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr rigid_bodies_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr unlabeled_markers_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    high_resolution_clock::time_point t_start;

    //Methods
    void sendFakeMessage();
    

public:
    // Definition of the construtors
    MoCapPublisher();

    // Send methods
    void sendRigidBodyMessage(double cameraMidExposureSecsSinceEpoch, sRigidBodyData* bodies_ptr, int nRigidBodies);
    void sendUnlabeledMarkersMessage(double cameraMidExposureSecsSinceEpoch, sMarker* markers_ptr, int nLabeledMarkers);
 
    // Getters
    std::string getServerAddress();
    std::string getLocalAddress();
    int getConnectionType();
    std::string getMulticastAddress();
    uint16_t getServerCommandPort();
    uint16_t getServerDataPort();
    bool isRecordingRequested();
    std::string getTakeName();
    
    // Setters

};
 
#endif