#include "online_calibration_tf_alg.h"

OnlineCalibrationTfAlgorithm::OnlineCalibrationTfAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

OnlineCalibrationTfAlgorithm::~OnlineCalibrationTfAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void OnlineCalibrationTfAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// OnlineCalibrationTfAlgorithm Public API
