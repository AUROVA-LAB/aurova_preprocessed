#include "dump_imu_data_for_calibration_with_imutk_alg.h"

DumpImuDataForCalibrationWithImutkAlgorithm::DumpImuDataForCalibrationWithImutkAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

DumpImuDataForCalibrationWithImutkAlgorithm::~DumpImuDataForCalibrationWithImutkAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void DumpImuDataForCalibrationWithImutkAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// DumpImuDataForCalibrationWithImutkAlgorithm Public API
