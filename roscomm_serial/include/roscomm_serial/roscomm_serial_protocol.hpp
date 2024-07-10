#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstring>       // memcpy
#include <type_traits>   // std::is_pod
#include <iomanip>       // std::hex, std::setw, std::setfill
#include "roscomm_serial/roscomm_serial.hpp"

namespace roscomm
{
  // packet field defined
  struct SerialPacket {
    uint8_t stx;  // STX 
    std::vector<uint8_t> data;
    uint8_t etx;  // ETX 
  };

  template<typename MsgType>
  class RosCommSerialProtocol {
    public:
      RosCommSerialProtocol(ros::NodeHandle& nh, std::string port, int baudrate, std::uint32_t timeout, int max_bufsize, uint8_t stx, uint8_t etx)
      : nh_(nh),
	port_(port),
	baudrate_(baudrate),
	timeout_(timeout),
	max_bufsize_(max_bufsize),
	stx_(stx),
	etx_(etx)
      { 
        SerialRos_ = std::make_unique<roscomm::RosCommSerial<MsgType>>(
			  nh_, 
                          std::bind(&RosCommSerialProtocol::HandleSerialWrite, this, std::placeholders::_1),
                          port_, baudrate_, timeout_, max_bufsize_);
	SerialRos_->Open();
      }

      void HandleSerialWrite(const typename MsgType::ConstPtr& msg);
      void HandleSerialRead();
      void ProcessReceived(std::vector<uint8_t>& buffer);
      void ProcessPacket(const std::vector<uint8_t>& packet);

    private:
      ros::NodeHandle nh_;
      std::unique_ptr<roscomm::RosCommSerial<MsgType>> SerialRos_;
      std::string port_;
      int baudrate_;
      std::uint32_t timeout_;
      int max_bufsize_;
      uint8_t stx_;
      uint8_t etx_;

      SerialPacket MakeSerialPacket(const typename MsgType::ConstPtr& msg, uint8_t stx, uint8_t etx);
  };

  // Write serial according to msg assigned
  template<typename MsgType>
  void RosCommSerialProtocol<MsgType>::HandleSerialWrite(const typename MsgType::ConstPtr& msg) {
    SerialPacket packet = MakeSerialPacket(msg, stx_, etx_);
    
    // print packet 
    std::cout << "from Message:";
    for (uint8_t byte : packet.data) {
      std::cout << " 0x" << std::hex << static_cast<int>(byte);
    }
    std::cout << std::dec << std::endl;
  
    // serial write
    if (SerialRos_->IsOpen()){ 
      std::vector<uint8_t> buffer;

      buffer.push_back(packet.stx); // Push STX

      const uint8_t* data_ptr = packet.data.data();
      size_t data_size = packet.data.size();
      buffer.insert(buffer.end(), data_ptr, data_ptr + data_size); // Push data

      buffer.push_back(packet.etx); // Push ETX
 
      std::cout << "Serialized Packet:";
      for (uint8_t byte : buffer) {
       std::cout << " 0x" << std::hex << static_cast<int>(byte);
      }
      std::cout << std::dec << std::endl;

      SerialRos_->WriteByteBlock(buffer.data(), buffer.size());
    }
    else std::cout << "[RosCommSerialProtocol] Serial is not opened." << std::endl;
  }

  template<typename MsgType>
  SerialPacket RosCommSerialProtocol<MsgType>::MakeSerialPacket(const typename MsgType::ConstPtr& msg, uint8_t stx, uint8_t etx) {
    SerialPacket packet;

    packet.stx = stx;
    packet.etx = etx;
    packet.data = msg->data; //TODO::expend to various data type vector

    return packet;
  }
  

  template<typename MsgType>
  void RosCommSerialProtocol<MsgType>::HandleSerialRead() {
    SerialRos_->ReadByteBlock();
    ProcessReceived(SerialRos_->buf_);
  }

  template<typename MsgType>
  void RosCommSerialProtocol<MsgType>::ProcessReceived(std::vector<uint8_t>& buffer) {
    std::vector<uint8_t> packet;

    while (!buffer.empty()) {
      auto stx_iter = std::find(buffer.begin(), buffer.end(), stx_); // Find STX in the buffer

      // If STX not found or exceeds max size, clear the buffer and exit
      if (stx_iter == buffer.end() || std::distance(stx_iter, buffer.end()) > max_bufsize_) {
        buffer.clear();
        break;
      }

      auto etx_iter = std::find(stx_iter, buffer.end(), etx_);  // Find ETX after STX

      // If ETX not found or exceeds max size, clear the buffer and exit
      if (etx_iter == buffer.end() || std::distance(stx_iter, etx_iter) > max_bufsize_) {
        buffer.clear();
        break;
      } 

      // Calculate the size of the packet
      size_t packet_size = std::distance(stx_iter, etx_iter) + 1;  // +1 to include ETX
      
      packet.assign(stx_iter, stx_iter + packet_size); // Extract the packet from the buffer
      ProcessPacket(packet); // Process the packet
      
      buffer.erase(stx_iter, stx_iter + packet_size); // Erase the processed packet from the buffer
    }
  }

  template<typename MsgType>
  void RosCommSerialProtocol<MsgType>::ProcessPacket(const std::vector<uint8_t>& packet) {
    std::cout << "Processed packet of size " << packet.size() << ":";
    
    // Print packet content
    for (uint8_t byte : packet) {
      std::cout << " 0x" << std::hex << static_cast<int>(byte);
    }
    std::cout << std::dec << std::endl;
    
    std::vector<uint8_t> data(packet.begin() + 1, packet.end() - 1);
    MsgType msg;
    msg.data = data; // Assign packet data to roscomm_serial message
	
   // Publish message
    SerialRos_->Publish(msg);
  }
}
