/****************************************************************************
# minas_client.cpp:  MINAS A5B EtherCAT Motor Controller                    #
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

#include <minas_control/minas_hardware_interface.h>
#include <getopt.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

// ethercat
#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>

namespace minas_control
{

bool MinasGxEtherCatManager::initSoem(const std::string& ifname) {
  // Copy string contents because SOEM library doesn't 
  // practice const correctness
  const static unsigned MAX_BUFF_SIZE = 1024;
  char buffer[MAX_BUFF_SIZE];
  size_t name_size = minas_control::MinasEtherCatManager::ifname_.size();
  if (name_size > sizeof(buffer) - 1) 
  {
    fprintf(stderr, "Ifname %s exceeds maximum size of %u bytes\n", minas_control::MinasEtherCatManager::ifname_.c_str(), MAX_BUFF_SIZE);
    return false;
  }
  std::strncpy(buffer, minas_control::MinasEtherCatManager::ifname_.c_str(), MAX_BUFF_SIZE);

  printf("Initializing etherCAT master\n");

  if (!ec_init(buffer))
  {
    fprintf(stderr, "Could not initialize ethercat driver\n");
    return false;
  }

  /* find and auto-config slaves */
  if (ec_config_init(FALSE) <= 0)
  {
    fprintf(stderr, "No slaves found on %s\n", minas_control::MinasEtherCatManager::ifname_.c_str());
    return false;
  }

  printf("SOEM found and configured %d slaves\n", ec_slavecount);
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
  {
    // MINAS-A5B Serial Man = 066Fh, ID = [5/D]****[0/4/8][0-F]*
    printf(" Man: %8.8x ID: %8.8x Rev: %8.8x ", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
    if(IF_MINAS(ec_slave[cnt])) {
      printf(" MINAS Drivers\n");
      minas_control::MinasEtherCatManager::slave_ids_.push_back(cnt);
      minas_control::MinasEtherCatManager::num_clients_++;
    }
    if(IF_OMRON_GX(ec_slave[cnt])) {
      printf(" OMRON GX Drivers\n");
      gx_control::GxEtherCatManager::slave_ids_.push_back(cnt);
      gx_control::GxEtherCatManager::num_clients_++;
    }
  }
  rx_pdo_no_.resize(ec_slavecount);
  tx_pdo_no_.resize(ec_slavecount);

  printf("Found %d MINAS Drivers\n", minas_control::MinasEtherCatManager::num_clients_);
  printf("Found %d OMRON GX Drivers\n", gx_control::GxEtherCatManager::num_clients_);


  /*
    SET PDO maping 4    SX-DSV02470 p.52
			Index	  Size(bit)	Name
    RxPDO (1603h)	6040h 00h 16 Controlword
			6060h 00h  8 Modes of operation
			6071h 00h 16 Target Torque
			6072h 00h 16 Max torque
			607Ah 00h 32 Target Position
			6080h 00h 32 Max motor speed
			60B8h 00h 16 Touch probe function
			60FFh 00h 32 Target Velocity
    TxPDO (1A03h)
			603Fh 00h 16 Error code
			6041h 00h 16 Statusword
			6061h 00h  8 Modes of operation display
			6064h 00h 32 Position actual value
			606Ch 00h 32 Velocity actual value
			6077h 00h 16 Torque actual value
			60B9h 00h 16 Touch probe status
			60BAh 00h 32 Touch probe pos1 pos val
			60FDh 00h 32 Digital inputs
   */

  if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_PRE_OP)
    {
      fprintf(stderr, "Could not set EC_STATE_PRE_OP\n");
      return false;
    }

  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      static int i = 0;
      if (! IF_MINAS(ec_slave[cnt])) continue;
      printf(" %d : MINAS slave_id = %d\n", i, minas_control::MinasEtherCatManager::slave_ids_[i]);
      i++;
      int ret = 0, l;
      uint8_t num_entries;
      l = sizeof(num_entries);
      ret += ec_SDOread(cnt, 0x1603, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
      printf(" RxPDO len = %d\n", num_entries);
      num_entries = 0;
      ret += ec_SDOwrite(cnt, 0x1603, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);
      // add position offset (60B0 / 00h / I32)
      uint32_t mapping;
      mapping = 0x60B00020;
      ret += ec_SDOwrite(cnt, 0x1603, 0x09, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
      //
      num_entries = 9;
      ret += ec_SDOwrite(cnt, 0x1603, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);
      ret += ec_SDOread(cnt, 0x1603, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
      printf(" TXPDO len = %d\n", num_entries);
    }

  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      static int i = 0;
      if (! IF_OMRON_GX(ec_slave[cnt])) continue;
      printf(" %d : OMRON GX slave_id = %d\n", i, gx_control::GxEtherCatManager::slave_ids_[i]);
      i++;
      int ret = 0, l;
      l = sizeof(rx_pdo_no_[cnt]);
      ret += ec_SDOread(cnt, 0x1c12, 0x00, FALSE, &l, &rx_pdo_no_[cnt], EC_TIMEOUTRXM);
      printf("  RxPDO len = %d\n", rx_pdo_no_[cnt]);
      ret += ec_SDOread(cnt, 0x1c13, 0x00, FALSE, &l, &tx_pdo_no_[cnt], EC_TIMEOUTRXM);
      printf("  TxPDO len = %d\n", tx_pdo_no_[cnt]);
    }

  // use PDO mapping 4
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      if (! IF_MINAS(ec_slave[cnt])) continue;
      int ret = 0, l;
      uint8_t num_pdo ;
      // set 0 change PDO mapping index
      num_pdo = 0;
      ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
      // set to default PDO mapping 4
      uint16_t idx_rxpdo = 0x1603;
      ret += ec_SDOwrite(cnt, 0x1c12, 0x01, FALSE, sizeof(idx_rxpdo), &idx_rxpdo, EC_TIMEOUTRXM);
      // set number of assigned PDOs
      num_pdo = 1;
      ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
      printf("RxPDO mapping object index %d = %04x ret=%d\n", cnt, idx_rxpdo, ret);

      // set 0 change PDO mapping index
      num_pdo = 0;
      ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
      // set to default PDO mapping 4
      uint16_t idx_txpdo = 0x1a03;
      ret += ec_SDOwrite(cnt, 0x1c13, 0x01, FALSE, sizeof(idx_txpdo), &idx_txpdo, EC_TIMEOUTRXM);
      // set number of assigned PDOs
      num_pdo = 1;
      ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
      printf("TxPDO mapping object index %d = %04x ret=%d\n", cnt, idx_txpdo, ret);
    }

  // configure IOMap
  int iomap_size = ec_config_map(minas_control::MinasEtherCatManager::iomap_);
  printf("SOEM IOMap size: %d\n", iomap_size);

  // locates dc slaves - ???
  ec_configdc();

  // '0' here addresses all slaves
  if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP)
  {
    fprintf(stderr, "Could not set EC_STATE_SAFE_OP\n");
    return false;
  }

  /* 
      This section attempts to bring all slaves to operational status. It does so
      by attempting to set the status of all slaves (ec_slave[0]) to operational,
      then proceeding through 40 send/recieve cycles each waiting up to 50 ms for a
      response about the status. 
  */
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  ec_writestate(0);
  int chk = 40;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000); // 50 ms wait for state check
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if(ec_statecheck(0,EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
  {
    fprintf(stderr, "OPERATIONAL state not set, exiting\n");
    return false;
  }

  ec_readstate();
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
	     cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
	     ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
      if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
      printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
	     (ec_slave[cnt].activeports & 0x02) > 0 , 
	     (ec_slave[cnt].activeports & 0x04) > 0 , 
	     (ec_slave[cnt].activeports & 0x08) > 0 );
      printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
    }

  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      int ret = 0, l;
      uint16_t sync_mode;
      uint32_t cycle_time;
      uint32_t minimum_cycle_time;
      uint32_t sync0_cycle_time;
      l = sizeof(sync_mode);
      ret += ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &l, &sync_mode, EC_TIMEOUTRXM);
      l = sizeof(cycle_time);
      ret += ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &l, &cycle_time, EC_TIMEOUTRXM);
      l = sizeof(minimum_cycle_time);
      ret += ec_SDOread(cnt, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time, EC_TIMEOUTRXM);
      l = sizeof(sync0_cycle_time);
      ret += ec_SDOread(cnt, 0x1c32, 0x0a, FALSE, &l, &sync0_cycle_time, EC_TIMEOUTRXM);
      printf("\nSlave:%d\n Name:%s\n", cnt, ec_slave[cnt].name);
      printf("  PDO syncmode %02x, cycle time %d ns (min %d), sync0 cycle time %d ns, ret = %d\n", sync_mode, cycle_time, minimum_cycle_time, sync0_cycle_time, ret);
    }

  printf("\nFinished configuration successfully\n");
  return true;
}

//

#define PULSE_PER_REVOLUTE ( (1048576 / (2 * M_PI) ) * 101 ) // 20 bit / 101 reduction
  //#define PULSE_PER_REVOLUTE ( ( 131072 / (2 * M_PI) ) * 101 )// 17 bit / 101 reduction

EtherCATJointControlInterface::EtherCATJointControlInterface(minas_control::MinasEtherCatManager* manager, int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd, int torque_for_emergency_stop, int over_load_level, int over_speed_level, double motor_working_range, int max_motor_speed, int max_torque) : JointControlInterface(slave_no, jnt_stat, jnt_cmd)
  {
    ROS_INFO("Initialize EtherCATJoint (%d)", slave_no);
    // EtherCAT
    int operation_mode = 0x08; // (csp) cyclic synchronous position mode

    client = new MinasClient(*manager, slave_no);

    ROS_INFO("Initialize EtherCATJoint (reset)");
    client->reset();

    // set paramete from PANATERM test program
    ROS_INFO("Initialize EtherCATJoint (TorqueForEmergencyStop %d)", torque_for_emergency_stop);
    client->setTrqueForEmergencyStop(torque_for_emergency_stop); // unit [%]
    ROS_INFO("Initialize EtherCATJoint (OverLoadLevel %d)", over_load_level);
    client->setOverLoadLevel(over_load_level);          // unit [%]
    ROS_INFO("Initialize EtherCATJoint (OverSpeedLevel %d)", over_speed_level);
    client->setOverSpeedLevel(over_speed_level);        // [r/min]
    ROS_INFO("Initialize EtherCATJoint (MotorWorkingRange %.1f)", motor_working_range);
    client->setMotorWorkingRange(motor_working_range);  // (unit 0.1, full range is 1)

    ROS_INFO("Initialize EtherCATJoint (InterpolationTimePeriod)");
    client->setInterpolationTimePeriod(4000);     // 4 msec

    // servo on
    ROS_INFO("Initialize EtherCATJoint (servoOn)");
    client->servoOn();

    // get current positoin
    ROS_INFO("Initialize EtherCATJoint (readInputs)");
    input = client->readInputs();
    int32 current_position = input.position_actual_value;

    ROS_INFO("Initialize EtherCATJoint (set target position)");
    // set target position
    memset(&output, 0x00, sizeof(minas_control::MinasOutput));
    if ( operation_mode == 0x01 )
      { // (pp) position profile mode
	output.target_position = (current_position > 0)?(current_position - 0x100000):(current_position + 0x100000);
      }
    else
      { // (csp) cyclic synchronous position mode
	//output.target_position = current_position;
	output.target_position = 0;
	output.position_offset = current_position;
      }
    output.max_motor_speed = max_motor_speed;  // rad/min
    output.target_torque = 0;    // 0% (unit 0.1%)
    output.max_torque    = max_torque;    // 50% (unit 0.1%)
    output.controlword   = 0x001f; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5)

    // change to cyclic synchronous position mode
    output.operation_mode = operation_mode;
    //output.operation_mode = 0x08; // (csp) cyclic synchronous position mode
    //output.operation_mode = 0x01; // (pp) position profile mode

    // set profile velocity
    ROS_INFO("Initialize EtherCATJoint (setProfileVelocity)");
    client->setProfileVelocity(0x20000000);

    ROS_INFO("Initialize EtherCATJoint (pp control model setup)");
    // pp control model setup (see statusword(6041.h) 3) p.107)
    client->writeOutputs(output);
    while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)
      input = client->readInputs();
    }
    ROS_INFO("Initialize EtherCATJoint (clear new set point)");
    output.controlword   &= ~0x0010; // clear new-set-point (bit4)
    client->writeOutputs(output);

    ROS_WARN("target position = %08x", output.target_position);
    ROS_WARN("position offset = %08x", output.position_offset);
    joint.cmd_ = joint.pos_ = current_position / (PULSE_PER_REVOLUTE);
    joint.vel_ = joint.eff_ = 0;
    ROS_INFO("Initialize EtherCATJoint .. done");
  }

  EtherCATJointControlInterface::~EtherCATJointControlInterface()
  {
    ROS_INFO_STREAM_NAMED("minas", "~EtherCATJointControlInterface()");
    shutdown();
    delete(client);
  }

  void EtherCATJointControlInterface::shutdown()
  {
    ROS_INFO_STREAM_NAMED("minas", joint.name_ + " shutdown()");
    client->printPDSStatus(input);
    client->printPDSOperation(input);
    client->reset();
    client->servoOff();
  }

  void EtherCATJointControlInterface::read()
  {
    input = client->readInputs();
    output = client->readOutputs();
    joint.pos_ = int32_t(input.position_actual_value) / PULSE_PER_REVOLUTE;
    joint.vel_ = int32_t(input.velocity_actual_value) / PULSE_PER_REVOLUTE;
    joint.eff_ = int32_t(input.torque_actual_value) / PULSE_PER_REVOLUTE;
  }

  void EtherCATJointControlInterface::write()
  {
    output.position_offset = uint32_t(joint.cmd_ * PULSE_PER_REVOLUTE);
    client->writeOutputs(output);
  }

  //
  DummyJointControlInterface::DummyJointControlInterface(int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd) : JointControlInterface(slave_no, jnt_stat, jnt_cmd) {
    joint.cmd_ = joint.pos_ = joint.vel_ = joint.eff_ = 0;
  }

  void DummyJointControlInterface::read()
  {
    joint.pos_ = joint.cmd_;
    joint.vel_ = 0;
    joint.eff_ = 0;
  }

  void DummyJointControlInterface::write()
  {
  }

  //
  void MinasHardwareInterface::getParamFromROS(int joint_no, int &torque_for_emergency_stop, int &over_load_level, int &over_speed_level, double &motor_working_range, int &max_motor_speed, int &max_torque)
  {
    std::string joint_name("~joint" + boost::lexical_cast<std::string>(joint_no));
    ros::param::param<int>(joint_name + "/torque_for_emergency_stop", torque_for_emergency_stop, 100); // 100%
    ros::param::param<int>(joint_name + "/over_load_level", over_load_level, 50); // 50%
    ros::param::param<int>(joint_name + "/over_speed_level", over_speed_level, 120); // r/min
    ros::param::param<double>(joint_name + "/motor_working_range", motor_working_range, 0.1); // 0.1
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/torque_for_emergency_stop : " << torque_for_emergency_stop);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/over_load_level           : " << over_load_level);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/over_speed_level          : " << over_speed_level);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/motor_working_range       : " << motor_working_range);
    ros::param::param<int>(joint_name + "/max_motor_speed", max_motor_speed, 120); // rad/min
    ros::param::param<int>(joint_name + "/max_torque", max_torque, 500); // 50% (unit 0.1%)
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/max_motor_speed           : " << max_motor_speed);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/max_torque                : " << max_torque);
  }

  MinasHardwareInterface::MinasHardwareInterface(std::string ifname, bool in_simulation)
    : manager(NULL)
  {
    /* start MinasClient */
    if (in_simulation) {
      ROS_INFO_STREAM_NAMED("minas","Minas Hardware Interface in simulation mode");
      for (int i = 1; i <= 6; i++ ) {
        int torque_for_emergency_stop, over_load_level, over_speed_level;
        double motor_working_range;
        int max_motor_speed, max_torque;
        getParamFromROS(i, torque_for_emergency_stop, over_load_level, over_speed_level, motor_working_range, max_motor_speed, max_torque);
	registerControl(new DummyJointControlInterface(i,
						       joint_state_interface,
						       joint_position_interface
						       ));
      }
    } else {
    
      manager = new minas_control::MinasGxEtherCatManager(ifname);
      manager->minas_control::MinasEtherCatManager::init();

      n_dof_ = manager->minas_control::MinasEtherCatManager::getNumClinets();
      n_dio_  = manager->gx_control::GxEtherCatManager::getNumClinets();
      ROS_INFO_STREAM_NAMED("minas", "Minas Hardware Interface found " << n_dof_ << " MINAS clients");
      ROS_INFO_STREAM_NAMED("minas", "Minas Hardware Interface found " << n_dio_ << " OMRON GX clients");
      if ( n_dof_ != 6 ) {
	ROS_ERROR_STREAM_NAMED("minas", "Minas Hardware Interface expecting 6 MINAS clients");
      }
      int i;
      for (i = 1; i <= n_dof_; i++ )
	{
          int torque_for_emergency_stop, over_load_level, over_speed_level;
          double motor_working_range;
          int max_motor_speed, max_torque;
          getParamFromROS(i, torque_for_emergency_stop, over_load_level, over_speed_level, motor_working_range, max_motor_speed, max_torque);
	  registerControl(new EtherCATJointControlInterface(
                                                            static_cast<minas_control::MinasEtherCatManager* >(manager), i,
							    joint_state_interface,
							    joint_position_interface,
                                                            torque_for_emergency_stop, over_load_level, over_speed_level, motor_working_range,
                                                            max_motor_speed, max_torque
                                                            ));
	}
      for (; i <= 6; i++ )
	{
	  registerControl(new DummyJointControlInterface(i,
							 joint_state_interface,
							 joint_position_interface
							 ));
	  ROS_ERROR_STREAM_NAMED("minas", "Could not find EtherCAT client");
	  ROS_ERROR_STREAM_NAMED("minas", "Minas Hardware Interface uses Dummy joint " << i);
	}
    }

    registerInterface(&joint_state_interface);
    registerInterface(&joint_position_interface);

    gx_driver_interface.registerHandle(hardware_interface::GxDriverHandle("gx_driver", static_cast<gx_control::GxEtherCatManager* >(manager)));
    registerInterface(&gx_driver_interface);
  }

  MinasHardwareInterface::~MinasHardwareInterface()
  {
    shutdown();
  }

  void MinasHardwareInterface::shutdown()
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->shutdown();
    }
    controls.clear();
    if ( manager != NULL ) {
      ROS_INFO_STREAM_NAMED("minas", "Delete manager");
      delete(manager);
    }
    manager = NULL;
  }

  void MinasHardwareInterface::registerControl(JointControlInterface* control)
  {
    controls.push_back(control);
  }

  bool MinasHardwareInterface::read(const ros::Time time, const ros::Duration period)
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->read();
    }
  }

  void MinasHardwareInterface::write(const ros::Time time, const ros::Duration period)
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->write();
    }
  }

  inline ros::Time MinasHardwareInterface::getTime()
  {
    return ros::Time::now();
  }

  inline ros::Duration MinasHardwareInterface::getPeriod()
  {
    return ros::Duration(0.001);
  }

} // namespace

