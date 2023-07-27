// agv_robot_info_class.h
#include "robot_info/robot_info.h"
#include <string>

class HydraulicSystemMonitor {
public:
  std::string hydraulic_oil_temperature;
  std::string hydraulic_oil_tank_fill_level;
  std::string hydraulic_oil_pressure;
 HydraulicSystemMonitor();
  std::string getTemperature();
  std::string getFillLevel();
  std::string getPressure();
};