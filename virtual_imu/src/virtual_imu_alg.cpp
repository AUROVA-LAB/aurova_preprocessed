#include "virtual_imu_alg.h"

VirtualImuAlgorithm::VirtualImuAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

VirtualImuAlgorithm::~VirtualImuAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void VirtualImuAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// VirtualImuAlgorithm Public API
