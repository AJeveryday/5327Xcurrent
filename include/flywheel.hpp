#include "okapi/api.hpp"
namespace flywheel {

    #define FLYWHEEL_PORT 19
    
    #define LONG_RANGE_POWER 0.6666667
    #define SHORT_RANGE_POWER 0.3333333
    #define MAXIMUM_VOLTAGE 12000
    #define TECHNICAL_FLYWHEEL_RPM 600

    extern okapi::Motor motor;

    extern int targetSpeed;
    extern double actualSpeed;

    int voltageUpdate();
    void setTargetSpeed(double pwr);

};