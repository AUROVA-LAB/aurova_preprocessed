#include "ackermann_to_odom_alg.h"

AckermannToOdomAlgorithm::AckermannToOdomAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

AckermannToOdomAlgorithm::~AckermannToOdomAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void AckermannToOdomAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// AckermannToOdomAlgorithm Public API
