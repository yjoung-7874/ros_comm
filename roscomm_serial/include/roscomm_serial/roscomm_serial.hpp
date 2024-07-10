#ifndef ROSCOMM_SERIAL_INCLUDED
#define ROSCOMM_SERIAL_INCLUDED

#include <ros/ros.h>
#include <serial/serial.h>
#include <optional>
#include "ros_wrapper/pubsub.hpp"

// -- TEST -- 
#include <std_msgs/String.h>

namespace roscomm 
{
  template<typename MsgType>
  class RosCommSerial {
    private:
      // Serial ---------------------
      serial::Serial ser_;

      // // Parameters
      std::string port_;
      int baudrate_; 
      std::uint32_t timeout_;

      // ROS ----------------------
      ros::NodeHandle nh_;

      // // Subscriber & Publisher
      ros_wrapper::Subscriber<MsgType> serial_write_sub_;
      std::function<void(const typename MsgType::ConstPtr&)> callback_; 
      ros_wrapper::Publisher<MsgType> serial_read_pub_;

    public:
      RosCommSerial();
      RosCommSerial(ros::NodeHandle& nh, std::function<void(const typename MsgType::ConstPtr&)> cb, 
                    std::string port, int baudrate=9600, std::uint32_t timeout=1000, int max_bufsize=1024);
      ~RosCommSerial();

      // Comm params
      void SetPort(std::string port);
      void SetBaudrate(int baudrate);
      void SetTimeout(std::uint32_t timeout);
      void SetSerialParams(std::string port, int baudrate, std::uint32_t timeout);

      // Comm functions
      void Open();
      void Close();
      
      bool IsOpen();
      std::size_t IsAvailable();
     
      std::vector<std::uint8_t> buf_;
      const int max_bufsize_;
      int cur_bufsize_;

      void WriteString();
      void WriteByte();
      void WriteByteBlock(const uint8_t* data, std::size_t size);
      
      std::optional<std::string> ReadString();
      void ReadByte();
      void ReadByteBlock();

      void Publish(MsgType msg);
  };    

  template<typename MsgType>
  RosCommSerial<MsgType>::RosCommSerial(ros::NodeHandle& nh,
					std::function<void(const typename MsgType::ConstPtr&)> cb,
					std::string port,
					int baudrate,
					std::uint32_t timeout,
					int max_bufsize)
  : nh_(nh),
    callback_(cb),
    port_(port),
    baudrate_(baudrate),
    timeout_(timeout),
    max_bufsize_(max_bufsize)
  {
    serial_write_sub_ = ros_wrapper::Subscriber<MsgType>(nh_, "serial_write", 10, callback_);
    serial_read_pub_ = ros_wrapper::Publisher<MsgType>(nh_, "serial_read", 10);

    std::cout << "RosCommSerial Constructed" << std::endl;
  }

  template<typename MsgType>
  RosCommSerial<MsgType>::~RosCommSerial() {
    std::cout << "RosCommSerial De-Constructed" << std::endl;
  }

  template<typename MsgType>
  void RosCommSerial<MsgType>::SetPort(std::string port) { port_ = port; }

  template<typename MsgType>
  void RosCommSerial<MsgType>::SetTimeout(std::uint32_t timeout) { timeout_ = timeout; }

  template<typename MsgType>
  void RosCommSerial<MsgType>::SetBaudrate(int baudrate) { baudrate_ = baudrate; }

  template<typename MsgType>
  void RosCommSerial<MsgType>::SetSerialParams(std::string port, int baudrate, std::uint32_t timeout) {
    port_ = port;
    baudrate_ = baudrate;
    timeout_ = timeout;
  }

  template<typename MsgType>
  void RosCommSerial<MsgType>::Open() {
    try {
      ser_.setPort(port_);
      ser_.setBaudrate(baudrate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
      ser_.setTimeout(to);
      ser_.open();
    } catch (serial::IOException& e) {
      ROS_INFO_STREAM("Unable to open port: " << port_ << ", baudrate: " << baudrate_ << ", timeout: " << timeout_);
      return;
    }

    if (ser_.isOpen()) ROS_INFO_STREAM("Serial Port Initialized");
  }

  template<typename MsgType>
  bool RosCommSerial<MsgType>::IsOpen() { return ser_.isOpen(); }

  template<typename MsgType>
  std::size_t RosCommSerial<MsgType>::IsAvailable() { return ser_.available(); }

  template<typename MsgType>
  std::optional<std::string> RosCommSerial<MsgType>::ReadString() {
    std::size_t _len_received = IsAvailable();

    std::optional<std::string> data;

    if (_len_received > 0) data = ser_.read(_len_received);
    else data = std::nullopt;

    return data;
  }

  template<typename MsgType>
  void RosCommSerial<MsgType>::ReadByteBlock() {
    std::size_t _len_received = IsAvailable();
    if (_len_received) {
      std::size_t _len_read = ser_.read(buf_, _len_received);
      cur_bufsize_ += _len_read;
    } 
  }

  template<typename MsgType>
  void RosCommSerial<MsgType>::Publish(MsgType msg) {
    serial_read_pub_.publish(msg);
  }

  template<typename MsgType>
  void RosCommSerial<MsgType>::WriteByteBlock(const uint8_t* data, std::size_t size) {
    if (ser_.isOpen()) ser_.write(data, size);
  }
}

#endif // ! ROSCOMM_SERIAL_INCLUDED
