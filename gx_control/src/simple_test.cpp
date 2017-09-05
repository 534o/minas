/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test -i [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 */

#include <stdio.h>
#include <ethercat_manager/ethercat_manager.h>
#include <gx_control/gx_client.h>
#include <getopt.h>
#include <time.h>

static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
      tick.tv_nsec -= NSEC_PER_SECOND;
      tick.tv_sec++;
    }
}

int main(int argc, char *argv[])
{
  int operation_mode = 0x01; // (pp) position profile mode
  std::string ifname;

  printf("OMRON GX Simple Test using SOEM (Simple Open EtherCAT Master)\n");
  while (1) {
    static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"interface", required_argument, 0, 'i'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "hi:", long_options, &option_index);
    if (c == -1) break;
    switch (c) {
    case 'h':
      fprintf(stderr, "Usage: simple_test [options]\n");
      fprintf(stderr, "  Available options\n");
      fprintf(stderr, "    -i, --interface     NIC interface name for EtherCAT network\n");
      fprintf(stderr, "    -h, --help          Print this message and exit\n");
      exit(0);
      break;
    case 'i':
      ifname = optarg;
      break;
    }
  }
  /* start slaveinfo */
  gx_control::GxEtherCatManager manager(ifname);
  manager.init();
  std::vector<gx_control::GxClient *> clients;
  for (int i = 0; i < manager.getNumClinets(); i++ )
    {
      printf("initializing gx clinet %d : slave_id_ %d\n", i, manager.getSlaveId(i));
      clients.push_back(new gx_control::GxClient(manager, manager.getSlaveId(i)));
    }

  for (std::vector<gx_control::GxClient*>::iterator it = clients.begin(); it != clients.end(); ++it)
    {
      gx_control::GxClient* client = (*it);
      // clear error
      client->reset();
      // client->setInputTimeConstant(1);
      //double tm = client->getInputTimeConstant();
      //printf(" Input time constant %f[ms]\n", tm);
    }

  double period = 2e+5;   // 20ms in nanoseconds
  //double period = 4e+6;   //  4ms in nanoseconds
  // get curren ttime
  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  timespecInc(tick, period);

  for (int i = 0; i <= 2000; i++ ) {
    for (std::vector<gx_control::GxClient*>::iterator it = clients.begin(); it != clients.end(); ++it)
      {
	gx_control::GxClient* client = (*it);
        printf("slave_id: %d\n", client->getSlaveId());
        if ( client->getInputBit() >  0 ) {
          gx_control::GxInput input = client->readInputs();
          printf(" input  (%d) : ", client->getInputBit());
          for (unsigned i = 0; i < client->getInputBit(); ++i)
          {
            printf("%d ", input.data[i]);
          }
          printf("\n");
        }
        
        if ( client->getOutputBit() >  0 ) {
          gx_control::GxOutput output = client->readOutputs();
          printf(" output (%d) : ", client->getOutputBit());
          for (unsigned i = 0; i < client->getOutputBit(); ++i)
          {
            printf("%d ", output.data[i]);
            output.data[i] = 0;
          }
          printf("\n");
          output.data[(i/10)%16] = 1;
          client->writeOutputs(output);
        }
      } // for clients

    //usleep(4*1000);
    timespecInc(tick, period);
    // check overrun
    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -  (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
    if (overrun_time > 0.0)
      {
	fprintf(stderr, "  overrun: %f [sec]\n", overrun_time);
      }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
  }

  printf("End program\n");

  return 0;
}

