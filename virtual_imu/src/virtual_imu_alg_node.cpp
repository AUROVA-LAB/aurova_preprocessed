#include "virtual_imu_alg_node.h"

VirtualImuAlgNode::VirtualImuAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<VirtualImuAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

VirtualImuAlgNode::~VirtualImuAlgNode(void)
{
  // [free dynamic memory]
}

void VirtualImuAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void VirtualImuAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void VirtualImuAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<VirtualImuAlgNode>(argc, argv, "virtual_imu_alg_node");
}
