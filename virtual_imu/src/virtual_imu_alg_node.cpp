#include "virtual_imu_alg_node.h"

VirtualImuAlgNode::VirtualImuAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<VirtualImuAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 100; //in [Hz]
  this->originl_imu_msg_.angular_velocity.x = 0.0;
  this->originl_imu_msg_.angular_velocity.y = 0.0;
  this->originl_imu_msg_.angular_velocity.z = 0.0;
  this->virtual_imu_msg_.angular_velocity.x = 0.0;
  this->virtual_imu_msg_.angular_velocity.y = 0.0;
  this->virtual_imu_msg_.angular_velocity.z = 0.0;

  // [init publishers]
  this->imu_publisher_ = this->public_node_handle_.advertise < sensor_msgs::Imu > ("/virtual_imu_data", 1);

  // [init subscribers]
  this->original_imu_ = this->public_node_handle_.subscribe("/imu/data", 1, &VirtualImuAlgNode::cb_imuData, this);

  XmlRpc::XmlRpcValue accMisalignMatrixConfig;
  if (this->public_node_handle_.hasParam("/acc_misalign_matrix"))
  {
    try
    {
      this->public_node_handle_.getParam("/acc_misalign_matrix", accMisalignMatrixConfig);

      ROS_ASSERT(accMisalignMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = acc_misaligment_.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << accMisalignMatrixConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> acc_misaligment_(i, j);
          }
          catch (XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch (...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM(
          "ERROR reading IMU calibration: " << e.getMessage() << " for misalign_matrix (type: "
              << accMisalignMatrixConfig.getType() << ")");
    }
    std::cout << "Loaded acc_misalign_matrix using rosparam: " << std::endl;
    std::cout << acc_misaligment_ << std::endl;
  }

  XmlRpc::XmlRpcValue accScaleMatrixConfig;
  if (this->public_node_handle_.hasParam("/acc_scale_matrix"))
  {
    try
    {
      this->public_node_handle_.getParam("/acc_scale_matrix", accScaleMatrixConfig);

      ROS_ASSERT(accScaleMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = acc_scale_factor_.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << accScaleMatrixConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> acc_scale_factor_(i, j);
          }
          catch (XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch (...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM(
          "ERROR reading IMU calibration: " << e.getMessage() << " for acc_scale_factor_matrix (type: "
              << accScaleMatrixConfig.getType() << ")");
    }
    std::cout << "Loaded acc_scale_factor_matrix using rosparam: " << std::endl;
    std::cout << acc_scale_factor_ << std::endl;
  }

  XmlRpc::XmlRpcValue accBiasVectorConfig;
  if (this->public_node_handle_.hasParam("/acc_bias_vector"))
  {
    try
    {
      this->public_node_handle_.getParam("/acc_bias_vector", accBiasVectorConfig);

      ROS_ASSERT(accBiasVectorConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = acc_bias_.rows();

      for (int i = 0; i < matSize; i++)
      {
        try
        {
          // These matrices can cause problems if all the types
          // aren't specified with decimal points. Handle that
          // using string streams.
          std::ostringstream ostr;
          ostr << accBiasVectorConfig[i];
          std::istringstream istr(ostr.str());
          istr >> acc_bias_(i);
        }
        catch (XmlRpc::XmlRpcException &e)
        {
          throw e;
        }
        catch (...)
        {
          throw;
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM(
          "ERROR reading IMU calibration: " << e.getMessage() << " for acc_bias_vector (type: "
              << accBiasVectorConfig.getType() << ")");
    }
    std::cout << "Loaded acc_bias_vector using rosparam: " << std::endl;
    std::cout << acc_bias_ << std::endl;
  }

  ////////////////////////////////// Loading Gyro calibration!!! //////////////////////////////////////////////

  XmlRpc::XmlRpcValue gyroMisalignMatrixConfig;
  if (this->public_node_handle_.hasParam("/gyro_misalign_matrix"))
  {
    try
    {
      this->public_node_handle_.getParam("/gyro_misalign_matrix", gyroMisalignMatrixConfig);

      ROS_ASSERT(gyroMisalignMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = gyro_misaligment_.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << gyroMisalignMatrixConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> gyro_misaligment_(i, j);
          }
          catch (XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch (...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM(
          "ERROR reading IMU calibration: " << e.getMessage() << " for misalign_matrix (type: "
              << gyroMisalignMatrixConfig.getType() << ")");
    }
    std::cout << "Loaded gyro_misalign_matrix using rosparam: " << std::endl;
    std::cout << gyro_misaligment_ << std::endl;
  }

  XmlRpc::XmlRpcValue gyroScaleMatrixConfig;
  if (this->public_node_handle_.hasParam("/gyro_scale_matrix"))
  {
    try
    {
      this->public_node_handle_.getParam("/gyro_scale_matrix", gyroScaleMatrixConfig);

      ROS_ASSERT(gyroScaleMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = gyro_scale_factor_.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << gyroScaleMatrixConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> gyro_scale_factor_(i, j);
          }
          catch (XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch (...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM(
          "ERROR reading IMU calibration: " << e.getMessage() << " for gyro_scale_factor_matrix (type: "
              << gyroScaleMatrixConfig.getType() << ")");
    }
    std::cout << "Loaded gyro_scale_factor_matrix using rosparam: " << std::endl;
    std::cout << gyro_scale_factor_ << std::endl;
  }

  XmlRpc::XmlRpcValue gyroBiasVectorConfig;
  if (this->public_node_handle_.hasParam("/gyro_bias_vector"))
  {
    try
    {
      this->public_node_handle_.getParam("/gyro_bias_vector", gyroBiasVectorConfig);

      ROS_ASSERT(gyroBiasVectorConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = gyro_bias_.rows();

      for (int i = 0; i < matSize; i++)
      {
        try
        {
          // These matrices can cause problems if all the types
          // aren't specified with decimal points. Handle that
          // using string streams.
          std::ostringstream ostr;
          ostr << gyroBiasVectorConfig[i];
          std::istringstream istr(ostr.str());
          istr >> gyro_bias_(i);
        }
        catch (XmlRpc::XmlRpcException &e)
        {
          throw e;
        }
        catch (...)
        {
          throw;
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM(
          "ERROR reading IMU calibration: " << e.getMessage() << " for gyro_bias_vector (type: "
              << gyroBiasVectorConfig.getType() << ")");
    }
    std::cout << "Loaded gyro_bias_vector using rosparam: " << std::endl;
    std::cout << gyro_bias_ << std::endl;
  }

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
  this->alg_.createVirtualImu(this->originl_imu_msg_, this->virtual_imu_msg_);

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->imu_publisher_.publish(this->virtual_imu_msg_);
}

/*  [subscriber callbacks] */
void VirtualImuAlgNode::cb_imuData(const sensor_msgs::Imu& Imu_msg)
{
  this->alg_.lock();

  gyro_reading_(0) = Imu_msg.angular_velocity.x;
  gyro_reading_(1) = Imu_msg.angular_velocity.y;
  gyro_reading_(2) = Imu_msg.angular_velocity.z;

  gyro_corrected_ = gyro_misaligment_ * gyro_scale_factor_ * (gyro_reading_ - gyro_bias_);

  //std::cout << "Gyro reading   = " << gyro_reading_ << std::endl
  //          << "Gyro corrected = " << gyro_corrected_ << std::endl;

  this->originl_imu_msg_.angular_velocity.x = gyro_corrected_(0);
  this->originl_imu_msg_.angular_velocity.y = gyro_corrected_(1);
  this->originl_imu_msg_.angular_velocity.z = gyro_corrected_(2);

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void VirtualImuAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void VirtualImuAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < VirtualImuAlgNode > (argc, argv, "virtual_imu_alg_node");
}
