#include "ackermann_to_odom_alg_node.h"

AckermannToOdomAlgNode::AckermannToOdomAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<AckermannToOdomAlgorithm>()
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

AckermannToOdomAlgNode::~AckermannToOdomAlgNode(void)
{
  // [free dynamic memory]
}

void AckermannToOdomAlgNode::mainNodeThread(void)
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

void AckermannToOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void AckermannToOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<AckermannToOdomAlgNode>(argc, argv, "ackermann_to_odom_alg_node");
}
