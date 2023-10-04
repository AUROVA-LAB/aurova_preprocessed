#include "kitti_preprocess_alg.h"

KittiPreprocessAlgorithm::KittiPreprocessAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

KittiPreprocessAlgorithm::~KittiPreprocessAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void KittiPreprocessAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// KittiPreprocessAlgorithm Public API
