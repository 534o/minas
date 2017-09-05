/****************************************************************************
# minas_client.cpp:  MINAS A5B EtherCAT Motor Controller                    #
# Copyright (C) 2016, 2016, Tokyo Opensource Robotics Kyokai Association    #
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

#include <stdio.h>
#include <minas_control/minas_client.h>
#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>

// An effort to keep the lines less than 100 char long
namespace minas_control
{

#define IF_MINAS(_ec_slave) (((int)_ec_slave.eep_man == 0x066f) & ((((0xf0000000&(int)ec_slave[cnt].eep_id)>>28) == 0x5) || (((0xf0000000&(int)ec_slave[cnt].eep_id)>>28) == 0xD)))

bool MinasEtherCatManager::initSoem(const std::string& ifname) {
  // Copy string contents because SOEM library doesn't 
  // practice const correctness
  const static unsigned MAX_BUFF_SIZE = 1024;
  char buffer[MAX_BUFF_SIZE];
  size_t name_size = ifname_.size();
  if (name_size > sizeof(buffer) - 1) 
  {
    fprintf(stderr, "Ifname %s exceeds maximum size of %u bytes\n", ifname_.c_str(), MAX_BUFF_SIZE);
    return false;
  }
  std::strncpy(buffer, ifname_.c_str(), MAX_BUFF_SIZE);

  printf("Initializing etherCAT master\n");

  if (!ec_init(buffer))
  {
    fprintf(stderr, "Could not initialize ethercat driver\n");
    return false;
  }

  /* find and auto-config slaves */
  if (ec_config_init(FALSE) <= 0)
  {
    fprintf(stderr, "No slaves found on %s\n", ifname_.c_str());
    return false;
  }

  printf("SOEM found and configured %d slaves\n", ec_slavecount);
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
  {
    // MINAS-A5B Serial Man = 066Fh, ID = [5/D]****[0/4/8][0-F]*
    printf(" Man: %8.8x ID: %8.8x Rev: %8.8x %s\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev, IF_MINAS(ec_slave[cnt])?" MINAS Drivers":"");
    if(IF_MINAS(ec_slave[cnt])) {
      num_clients_++;
    }
  }
  printf("Found %d MINAS Drivers\n", num_clients_);


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

  // extend PDO mapping 4 see p. 53 of SX-DSV02470
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      if (! IF_MINAS(ec_slave[cnt])) continue;
      int ret = 0, l;
      uint8_t num_entries;
      l = sizeof(num_entries);
      ret += ec_SDOread(cnt, 0x1603, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
      printf("len = %d\n", num_entries);
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
      printf("len = %d\n", num_entries);
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
  int iomap_size = ec_config_map(iomap_);
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
      if (! IF_MINAS(ec_slave[cnt])) continue;
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
      if (! IF_MINAS(ec_slave[cnt])) continue;
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
      printf("PDO syncmode %02x, cycle time %d ns (min %d), sync0 cycle time %d ns, ret = %d\n", sync_mode, cycle_time, minimum_cycle_time, sync0_cycle_time, ret);
    }

  printf("\nFinished configuration successfully\n");
  return true;
}


MinasClient::MinasClient(minas_control::MinasEtherCatManager& manager, int slave_no)
  : manager_(manager)
  , slave_no_(slave_no)
{}

void MinasClient::writeOutputs(const MinasOutput& output)
{
  // Default PDO Mapping 4 168 bit = 21  byte
  // PDO Mapping 4 + offset 200 bit = 25  byte
  uint8_t map[25] = {0}; // array containing all 15 output registers

  map[0] = (output.controlword) & 0x00ff;
  map[1] = (output.controlword >> 8) & 0x00ff;
  map[2] = output.operation_mode;
  map[3] = (output.target_torque) & 0x00ff;
  map[4] = (output.target_torque >>  8) & 0x00ff;
  map[5] = (output.max_torque) & 0x00ff;
  map[6] = (output.max_torque >>  8) & 0x00ff;
  map[7] = (output.target_position) & 0x00ff;
  map[8] = (output.target_position >>  8) & 0x00ff;
  map[9] = (output.target_position >> 16) & 0x00ff;
  map[10] = (output.target_position >> 24) & 0x00ff;
  map[11] = (output.max_motor_speed) & 0x00ff;
  map[12] = (output.max_motor_speed >>  8) & 0x00ff;
  map[13] = (output.max_motor_speed >> 16) & 0x00ff;
  map[14] = (output.max_motor_speed >> 24) & 0x00ff;
  map[15] = (output.touch_probe_function) & 0x00ff;
  map[16] = (output.touch_probe_function >> 8) & 0x00ff;
  map[17] = (output.target_velocity) & 0x00ff;
  map[18] = (output.target_velocity >>  8) & 0x00ff;
  map[19] = (output.target_velocity >> 16) & 0x00ff;
  map[20] = (output.target_velocity >> 24) & 0x00ff;
  map[21] = (output.position_offset) & 0x00ff;
  map[22] = (output.position_offset >>  8) & 0x00ff;
  map[23] = (output.position_offset >> 16) & 0x00ff;
  map[24] = (output.position_offset >> 24) & 0x00ff;

  for (unsigned i = 0; i < 25; ++i)
  {
    manager_.write(slave_no_, i, map[i]);
  }
}

MinasInput MinasClient::readInputs() const
{
  MinasInput input;
  uint8_t map[25];
  for (unsigned i = 0; i < 25; ++i)
  {
    map[i] = manager_.readInput(slave_no_, i);
  }

  input.error_code			= *(uint16 *)(map+0);
  input.statusword			= *(uint16 *)(map+2);
  input.operation_mode			= *(uint8  *)(map+4);
  input.position_actual_value		= *(uint32 *)(map+5);
  input.velocity_actual_value		= *(uint32 *)(map+9);
  input.torque_actual_value		= *(uint16 *)(map+13);
  input.touch_probe_status		= *(uint16 *)(map+15);
  input.touch_probe_posl_pos_value	= *(uint32 *)(map+17);
  input.digital_inputs			= *(uint32 *)(map+21);

  if (input.error_code >> 8 == 0xff) {
    int ecode = (input.error_code)&0x00ff;
    printf("%s : %d ", (ecode<0x9f)?"ALARM":"ERROR", ecode);
    for (unsigned i=0; i<sizeof(error_map)/sizeof(error_map[0]); i++) {
      if (error_map[i].code == 99 || error_map[i].code == ecode) {
        printf("%s", error_map[i].text);
        break;
      }
    }
    printf("\n");
  }
  return input;
}

MinasOutput MinasClient::readOutputs() const
{

  MinasOutput output;

  uint8_t map[25];
  for (unsigned i = 0; i < 25; ++i)
  {
    map[i] = manager_.readOutput(slave_no_, i);
  }

  output.controlword			= *(uint16 *)(map+0);
  output.operation_mode			= *(uint8  *)(map+2);
  output.target_torque			= *(uint16 *)(map+3);
  output.max_torque			= *(uint16 *)(map+5);
  output.target_position		= *(uint32 *)(map+7);
  output.max_motor_speed		= *(uint32 *)(map+11);
  output.touch_probe_function		= *(uint16 *)(map+15);
  output.target_velocity		= *(uint32 *)(map+17);
  output.position_offset		= *(uint32 *)(map+21);

  return output;
}

void MinasClient::reset()
{
  MinasInput input = readInputs();
  if ( input.error_code == 0 ) return;

  // section 8.4 of SX-DSV02470 --248--
  MinasOutput output;
  memset(&output, 0x00, sizeof(MinasOutput));
  output.controlword = 0x0080; // fault reset
  output.operation_mode = 0x01; // position profile mode
  writeOutputs(output);
  input = readInputs();

  int loop = 0;
  while ( input.error_code != 0 ) {
    if ( loop++ % 100 == 1 ) {
      printf("error_code = %04x, status_word %04x, operation_mode = %2d, position = %08x\n",
	   input.error_code, input.statusword, input.operation_mode, input.position_actual_value);
      printf("Waiting for Fault Reset...\n");
    }
    usleep(10*1000);
    input = readInputs();
  }
  printf("Fault was cleared\n");
}

void MinasClient::servoOn()
{
  MinasInput input = readInputs();
  printPDSStatus(input);
  MinasOutput output;
  memset(&output, 0x00, sizeof(MinasOutput));
  output.operation_mode = 1; // pp (profile position mode)
  int loop = 0;
  while (getPDSStatus(input) != OPERATION_ENABLED) {
    switch ( getPDSStatus(input) ) {
      case SWITCH_DISABLED:
	output.controlword = 0x0006; // move to ready to switch on
	break;
      case READY_SWITCH:
	output.controlword = 0x0007; // move to switched on
	break;
      case SWITCHED_ON:
	output.controlword = 0x000f; // move to operation enabled
	break;
      case OPERATION_ENABLED:
	break;
      default:
	printf("unknown status");
	return;
      }
    writeOutputs(output);
    usleep(10*1000);
    input = readInputs();
    if (loop++ % 100 == 1) printPDSStatus(input);
  }
}

void MinasClient::servoOff()
{
  MinasInput input = readInputs();
  printPDSStatus(input);
  MinasOutput output;
  memset(&output, 0x00, sizeof(MinasOutput));
  int loop = 0;
  while (getPDSStatus(input) != SWITCH_DISABLED) {
    switch ( getPDSStatus(input) ) {
      case READY_SWITCH:
	output.controlword = 0x0000; // disable voltage
	break;
      case SWITCHED_ON:
	output.controlword = 0x0006; // shutdown
	break;
      case OPERATION_ENABLED:
	output.controlword = 0x0007; // disable operation
	break;
      default:
	printf("unknown status");
	output.controlword = 0x0000; // disable operation
	break;
    }
    writeOutputs(output);
    usleep(10*1000);
    input = readInputs();
    if (loop++ % 100 == 1) printPDSStatus(input);
  }
}

PDS_STATUS MinasClient::getPDSStatus(const MinasInput input) const
{
  uint16 statusword = input.statusword;
  if (((statusword) & 0x004f) == 0x0000) { // x0xx 0000
    return NOT_READY;
  }else if (((statusword) & 0x004f) == 0x0040) { // x1xx 0000
    return SWITCH_DISABLED;
  }else if (((statusword) & 0x006f) == 0x0021) { // x01x 0001
    return READY_SWITCH;
  }else if (((statusword) & 0x006f) == 0x0023) { // x01x 0011
    return SWITCHED_ON;
  }else if (((statusword) & 0x006f) == 0x0027) { // x01x 0111
    return OPERATION_ENABLED;
  }else if (((statusword) & 0x006f) == 0x0007) { // x00x 0111
    return QUICK_STOP;
  }else if (((statusword) & 0x004f) == 0x000f) { // x0xx 1111
    return FAULT_REACTION;
  }else if (((statusword) & 0x004f) == 0x0008) { // x0xx 1000
    return FAULT;
  }else{
    return UNKNOWN;
  }
}

PDS_OPERATION MinasClient::getPDSOperation(const MinasInput input) const
{
  int8 operation_mode = input.operation_mode;
  switch (operation_mode) {
  case 0: return NO_MODE_CHANGE;	break;
  case 1: return PROFILE_POSITION_MODE;	break; // pp
  case 2: return VELOCITY_MODE;		break; // vl
  case 3: return PROFILE_VELOCITY_MODE;	break; // pv
  case 4: return TORQUE_PROFILE_MODE;	break; // tq
  case 6: return HOMING_MODE;		break; // hm
  case 7: return INTERPOLATED_POSITION_MODE;	break; // ip
  case 8: return CYCLIC_SYNCHRONOUS_POSITION_MODE;	break; // csp
  case 9: return CYCLIC_SYNCHRONOUS_VELOCITY_MODE;	break; // csv
  case 10: return CYCLIC_SYNCHRONOUS_TORQUE_MODE;	break; // cst
  }
}

PDS_STATUS MinasClient::getPDSControl(const MinasInput input) const
{
  uint16 statusword = input.statusword;
}

void MinasClient::printPDSStatus(const MinasInput input) const
{
  printf("Statusword(6041h): %04x\n ", input.statusword);
  switch ( getPDSStatus(input) ) {
    case NOT_READY:
      printf("Not ready to switch on\n");
      break;
    case SWITCH_DISABLED:
      printf("Switch on disabled\n");
      break;
    case READY_SWITCH:
      printf("Ready to switch on\n");
      break;
    case SWITCHED_ON:
      printf("Switched on\n");
      break;
    case OPERATION_ENABLED:
      printf("Operation enabled\n");
      break;
    case QUICK_STOP:
      printf("Quick stop active\n");
      break;
    case FAULT_REACTION:
      printf("Fault reaction active\n");
      break;
    case FAULT:
      printf("Fault\n");
      break;
    case UNKNOWN:
      printf("Unknown status %04x\n", input.statusword);
      break;
    }
  if ( input.statusword & 0x0800 ) {
    printf(" Internal limit active\n");
  }
  switch ( getPDSOperation(input) ) {
  case PROFILE_POSITION_MODE:
    if ( (input.statusword & 0x3400) | 0x2000 ) {
      printf(" Following error\n");
    }
    if ( (input.statusword & 0x3400) | 0x1000 ) {
      printf(" Set-point acknowledge\n");
    }
    if ( (input.statusword & 0x3400) | 0x0400 ) {
      printf(" Target reached\n");
    }
    break;
  case VELOCITY_MODE:
    break;
  case PROFILE_VELOCITY_MODE:
    if ( (input.statusword & 0x3400) | 0x2000 ) {
      printf(" Max slippage error (Not supported)\n");
    }
    if ( (input.statusword & 0x3400) | 0x1000 ) {
      printf(" Speed\n");
    }
    if ( (input.statusword & 0x3400) | 0x0400 ) {
      printf(" Target reached\n");
    }
    break;
  case TORQUE_PROFILE_MODE:
    if ( (input.statusword & 0x3400) | 0x0400 ) {
      printf(" Target reached\n");
    }
    break;
  case HOMING_MODE:
    if ( (input.statusword & 0x3400) | 0x2000 ) {
      printf(" Homing error\n");
    }
    if ( (input.statusword & 0x3400) | 0x1000 ) {
      printf(" Homing attained\n");
    }
    if ( (input.statusword & 0x3400) | 0x0400 ) {
      printf(" Target reached\n");
    }
    break;
  case INTERPOLATED_POSITION_MODE:
    if ( (input.statusword & 0x3400) | 0x1000 ) {
      printf(" Ip mode active\n");
    }
    if ( (input.statusword & 0x3400) | 0x0400 ) {
      printf(" Target reached\n");
    }
    break;
  case CYCLIC_SYNCHRONOUS_POSITION_MODE:
    if ( (input.statusword & 0x3400) | 0x2000 ) {
      printf(" Following error\n");
    }
    if ( (input.statusword & 0x3400) | 0x1000 ) {
      printf(" Drive follows command value\n");
    }
    break;
  case CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
    if ( (input.statusword & 0x3400) | 0x1000 ) {
      printf(" Drive follows command value\n");
    }
    break;
  case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
    if ( (input.statusword & 0x3400) | 0x1000 ) {
      printf(" Drive follows command value\n");
    }
    break;
  }
}

void MinasClient::printPDSOperation(const MinasInput input) const
{
  printf("Mode of operation(6061h): %04x\n ", input.operation_mode);
  switch ( getPDSOperation(input) ) {
  case NO_MODE_CHANGE:
    printf("No mode change / no mode assigned\n");
    break;
  case PROFILE_POSITION_MODE:
    printf("Profile position mode\n");
    break;
  case VELOCITY_MODE:
    printf("Velocity mode\n");
    break;
  case PROFILE_VELOCITY_MODE:
    printf("Profile velocity mode\n");
    break;
  case TORQUE_PROFILE_MODE:
    printf("Torque profile mode\n");
    break;
  case HOMING_MODE:
    printf("Homing mode\n");
    break;
  case INTERPOLATED_POSITION_MODE:
    printf("Interpolated position mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_POSITION_MODE:
    printf("Cyclic synchronous position mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
    printf("Cyclic synchronous velocity mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
    printf("Cyclic synchronous torque mode\n");
    break;
  default:
    printf("Reserved %04x\n", input.operation_mode);
    break;
  }
}

void MinasClient::printPDSControl(const MinasInput input) const
{
}

void MinasClient::setTrqueForEmergencyStop(double val)
{
  // 3511h, unit: %, range: 0 - 500, I16
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3511, 0x00, i16val);
}

void MinasClient::setOverLoadLevel(double val)
{
  // 3512h, unit: %, range: 0 - 500, I16
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3512, 0x00, i16val);
}

void MinasClient::setOverSpeedLevel(double val)
{
  // 3513h, unit: r/min, range: 0 - 20000, I16
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3513, 0x00, i16val);
}

void MinasClient::setMotorWorkingRange(double val)
{
  // 3514h, unit: 0.1 revolute, range: 0 - 1000, I16
  int16_t i16val = (int16_t)(val*10);
  manager_.writeSDO<int16_t>(slave_no_, 0x3514, 0x00, i16val);
}

void MinasClient::setProfileVelocity(uint32_t val)
{
  // 6091h, unit: pulse, range: 0 - 4294967295, U32
  uint32_t u32val = (uint32_t)val;
  manager_.writeSDO<uint32_t>(slave_no_, 0x6081, 0x00, u32val);
}

void MinasClient::setInterpolationTimePeriod(int us)
{
  uint32_t u32val;
  uint8_t u8val;
  switch ( us ) {
  case  250: u32val =  250000; u8val = 25; break;
  case  500: u32val =  500000; u8val =  5; break;
  case 1000: u32val = 1000000; u8val =  1; break;
  case 2000: u32val = 2000000; u8val =  2; break;
  case 4000: u32val = 4000000; u8val =  4; break;
  default:
    fprintf(stderr, "setInterpolatinTimePeriod(%d) must be ether of 250, 500, 1000, 2000, 4000\n", us);
    return;
  }
  int ret = 0;
  ret += manager_.writeSDO<uint32_t>(slave_no_, 0x1c32, 0x02, u32val);
  ret += manager_.writeSDO<uint8_t>(slave_no_, 0x60c2, 0x01, u8val);
  printf("Set interpolation time period %d us (%d/%d)\n", us, u32val, u8val, ret);

  u32val = manager_.readSDO<uint32_t>(slave_no_, 0x1c32, 0x02);
  u8val = manager_.readSDO<uint8_t>(slave_no_, 0x60c2, 0x01);
  printf("1c32h: cycle time %d\n", u32val);
  printf("60c2h: interpolation time period value %d\n", u8val);
}

} // end of minas_control namespace
