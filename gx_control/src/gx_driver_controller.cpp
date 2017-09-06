/****************************************************************************
# gx_client.cpp:  OMRON GX EtherCAT Motor Controller                        #
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

#include <gx_control/gx_driver_controller.h>
#include <getopt.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace gx_control
{

GxDriverController::GxDriverController()
  : initialized_(false)
{}

bool GxDriverController::init(hardware_interface::GxDriverInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
{
  ROS_ASSERT(hw);

  try {
    GxEtherCatManager* manager = hw->getHandle("gx_driver").getManager();
    for (int i = 0; i < manager->getNumClinets(); i++ )
    {
      ROS_INFO("Initializing gx clinet %d : slave_id_ %d", i, manager->getSlaveId(i));
      clients_.push_back(new gx_control::GxClient(*manager, manager->getSlaveId(i)));
    }
  }
  catch(const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Could not find gx hardware manager: " << e.what());
    return false;
  }

  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  if (!controller_nh.getParam("prefix", prefix_))
  {
    ROS_ERROR("Parameter 'prefix' not set");
    return false;
  }

  nh_prefix_ = ros::NodeHandle(root_nh, prefix_);

  return true;
}

void GxDriverController::starting(const ros::Time& time)
{
}

void GxDriverController::update(const ros::Time& time, const ros::Duration& period)
{
  if ( !initialized_) {
    int i = 0;
    for (std::vector<gx_control::GxClient*>::iterator it = clients_.begin(); it != clients_.end(); ++it)
    {
      gx_control::GxClient* client = (*it);
      if ( client->getInputBit() >  0 ) {
        boost::shared_ptr<GxDriverPublisher> gx_driver_publisher;
        gx_driver_publisher.reset(new GxDriverPublisher(client, publish_rate_, nh_prefix_));
        gx_driver_publisher->init(time, "input" + boost::lexical_cast<std::string>(i++));
        gx_driver_publishers_.push_back(gx_driver_publisher);
      }
      if ( client->getOutputBit() >  0 ) {
        boost::shared_ptr<GxDriverSubscriber> gx_driver_subscriber;
        gx_driver_subscriber.reset(new GxDriverSubscriber(client,nh_prefix_));
        gx_driver_subscriber->init(time, "output" + boost::lexical_cast<std::string>(i++));
        gx_driver_subscribers_.push_back(gx_driver_subscriber);
      }
      initialized_ = true;
    }
  } // initialized_
  else
  {
    for (std::vector<boost::shared_ptr<GxDriverPublisher> >::iterator it = gx_driver_publishers_.begin(); it != gx_driver_publishers_.end(); ++it)
    {
      (*it)->update(time, period);
    }
  }
}

void GxDriverController::stopping(const ros::Time& /*time*/)
{
  // remove initialized flag to permit data type change and time resetting
  initialized_ = false;
}

//
GxDriverPublisher::GxDriverPublisher(gx_control::GxClient* client, double publish_rate,
                                     ros::NodeHandle nh_prefix)
{
  client_ = client;
  publish_rate_ = publish_rate;
  nh_prefix_ = nh_prefix;
}

void GxDriverPublisher::init(const ros::Time& time, std::string topic_name)
{
  // initialize time
  last_publish_time_ = time;

  // realtime publisher
  ROS_INFO_STREAM("Publish DigitalIO with " << topic_name);
  gx_input_realtime_pub_ = GxInputPublisherPtr(
    new realtime_tools::RealtimePublisher<gx_control::DigitalIO>(nh_prefix_, topic_name, 4));
  gx_input_realtime_pub_->lock();
  gx_input_realtime_pub_->msg_.header.stamp = ros::Time::now();
  gx_input_realtime_pub_->msg_.data.resize(client_->getInputBit());
  gx_input_realtime_pub_->unlock();
}

void GxDriverPublisher::update(const ros::Time& time, const ros::Duration& period)
{
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    // try to publish
    if (gx_input_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
      // populate message
      gx_input_realtime_pub_->msg_.header.stamp = time;
       for (unsigned i = 0; i < client_->getInputBit(); i++)
       {
         gx_control::GxInput input = client_->readInputs();
         gx_input_realtime_pub_->msg_.data[i] = input.data[i];
       }
      gx_input_realtime_pub_->unlockAndPublish();
    }
  }
}


GxDriverSubscriber::GxDriverSubscriber(gx_control::GxClient* client, ros::NodeHandle nh_prefix)
{
  client_ = client;
  nh_prefix_ = nh_prefix;
}

void GxDriverSubscriber::init(const ros::Time& time, std::string topic_name)
{
  ROS_INFO_STREAM("Subscribe DigitalIO with " << topic_name);
  gx_output_realtime_sub_ = nh_prefix_.subscribe(topic_name, 1, &GxDriverSubscriber::cb, this);
}

void GxDriverSubscriber::cb(const gx_control::DigitalIOPtr& msg)
{
  std::stringstream ss;
  for(unsigned int i = 0; i < msg->data.size(); i++ ) {
    ss << (msg->data[i]?"1":"0") << " ";
  }
  ROS_INFO_STREAM("Received DigitalIO msg " << ss.str());
  if ( msg->data.size() != client_->getOutputBit() )
  {
    ROS_ERROR_STREAM("Length of received DigitalIO msg is not equal to GX Driver Output ports " << msg->data.size() << " != " <<  client_->getOutputBit());
    return;
  }
  gx_control::GxOutput output = client_->readOutputs();
  for (unsigned i = 0; i < msg->data.size(); ++i)
  {
    output.data[i] = msg->data[i];
  }
  client_->writeOutputs(output);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(gx_control::GxDriverController, controller_interface::ControllerBase)
