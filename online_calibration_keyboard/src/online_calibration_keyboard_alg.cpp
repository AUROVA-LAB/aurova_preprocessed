#include "online_calibration_keyboard_alg.h"

OnlineCalibrationKeyboardAlgorithm::OnlineCalibrationKeyboardAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

OnlineCalibrationKeyboardAlgorithm::~OnlineCalibrationKeyboardAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void OnlineCalibrationKeyboardAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// OnlineCalibrationKeyboardAlgorithm Public API

int OnlineCalibrationKeyboardAlgorithm::getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

void OnlineCalibrationKeyboardAlgorithm::mapKeysToVelocities(int key, struct Twist& st_twist_change_calib)
{

  st_twist_change_calib.x = 0.0;
  st_twist_change_calib.y = 0.0;
  st_twist_change_calib.z = 0.0;
  st_twist_change_calib.r = 0.0;
  st_twist_change_calib.p = 0.0;
  st_twist_change_calib.w = 0.0;
  st_twist_change_calib.flag_data = true;

  float step_pos = 0.01;
  float step_ang = 0.01;

  switch (key)
  {
    case '1':
      st_twist_change_calib.x = step_pos;
      break;
    case 'q':
      st_twist_change_calib.x = -step_pos;
      break;
    case '2':
      st_twist_change_calib.y = step_pos;
      break;
    case 'w':
      st_twist_change_calib.y = -step_pos;
      break;
    case '3':
      st_twist_change_calib.z = step_pos;
      break;
    case 'e':
      st_twist_change_calib.z = -step_pos;
      break;
    case '4':
      st_twist_change_calib.r = step_ang;
      break;
    case 'r':
      st_twist_change_calib.r = -step_ang;
      break;
    case '5':
      st_twist_change_calib.p = step_ang;
      break;
    case 't':
      st_twist_change_calib.p = -step_ang;
      break;
    case '6':
      st_twist_change_calib.w = step_ang;
      break;
    case 'y':
      st_twist_change_calib.w = -step_ang;
      break;
    default:
      st_twist_change_calib.flag_data = false;
      break;

  }

  return;
}
