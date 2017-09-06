/****************************************************************************
# gx_driver_controller.h:  OMRON GX EtherCAT Motor Controller               #
# Copyright (C) 2017, Tokyo Opensource Robotics Kyokai Association          #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/

#ifndef GX_CONTROL_GX_HARDWARE_INTERFACE_
#define GX_CONTROL_GX_HARDWARE_INTERFACE_

// ROS
#include <ros/ros.h>

// ros_control
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

// gx
#include <gx_control/gx_client.h>

// msg
#include <gx_control/DigitalIO.h>

namespace hardware_interface
{
class GxDriverHandle
{
public:
  GxDriverHandle(const std::string& name, gx_control::GxEtherCatManager* manager)
    : name_(name), manager_(manager)
  {
    if (!manager_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. GxDriver manager is null.");
    }

  }

  std::string getName() const {return name_;}
  gx_control::GxEtherCatManager* getManager() const {return manager_;}
private:
  std::string name_;
  gx_control::GxEtherCatManager* manager_;
};

class GxDriverInterface : public HardwareResourceManager<GxDriverHandle> {};
} // namespace


namespace gx_control
{

class GxDriverPublisher
{
public:
  GxDriverPublisher(gx_control::GxClient* client, double publish_rate,
                    ros::NodeHandle nh_prefix);
  ~GxDriverPublisher() {}
  void init(const ros::Time& time, std::string topic_name);
  void update(const ros::Time& time, const ros::Duration& period);

protected:
  gx_control::GxClient* client_;
  ros::Time last_publish_time_;
  double publish_rate_;
  ros::NodeHandle nh_prefix_;
  std::string prefix_;
  typedef realtime_tools::RealtimePublisher<gx_control::DigitalIO> GxInputPublisher;
  typedef boost::shared_ptr<GxInputPublisher > GxInputPublisherPtr;
  GxInputPublisherPtr gx_input_realtime_pub_;
};

class GxDriverSubscriber
{
public:
  GxDriverSubscriber(gx_control::GxClient* client, ros::NodeHandle nh_prefix);
  ~GxDriverSubscriber() {}
  void init(const ros::Time& time, std::string topic_name);
  void cb(const gx_control::DigitalIOPtr& msg);

protected:
  gx_control::GxClient* client_;
  ros::NodeHandle nh_prefix_;
  std::string prefix_;
  ros::Subscriber gx_output_realtime_sub_;
};

// this controller gets access to the GxDriverInterface
class GxDriverController: public controller_interface::Controller<hardware_interface::GxDriverInterface>
{
 public:
  GxDriverController();
  bool init(hardware_interface::GxDriverInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& /*period*/);
  void stopping(const ros::Time& /*time*/);

private:
  std::vector<gx_control::GxClient *> clients_;
  double publish_rate_;
  ros::NodeHandle nh_prefix_;
  std::string prefix_;
  bool initialized_;
  std::vector<boost::shared_ptr<GxDriverPublisher> > gx_driver_publishers_;
  std::vector<boost::shared_ptr<GxDriverSubscriber> > gx_driver_subscribers_;

};

} // namespace

#endif // GX_CONTROL_GX_HARDWARE_INTERFACE_
