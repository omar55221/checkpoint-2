#include "robot_info/hydraulic_system_monitor.h"


HydraulicSystemMonitor::HydraulicSystemMonitor() {
  hydraulic_oil_temperature = "45C";
  hydraulic_oil_tank_fill_level = "100%";
  hydraulic_oil_pressure = "250 bar";
}

std::string HydraulicSystemMonitor::getTemperature() {
  return hydraulic_oil_temperature;
}

std::string HydraulicSystemMonitor::getFillLevel() {
  return hydraulic_oil_tank_fill_level;
}

std::string HydraulicSystemMonitor::getPressure() {
  return hydraulic_oil_pressure;
}
