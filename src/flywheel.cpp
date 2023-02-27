#include "main.h"




#define FLYWHEEL_GEARSET pros::E_MOTOR_GEARSET_06
#define INTAKE_GEARSET pros::E_MOTOR_GEARSET_06
namespace flywheel{
    
    okapi::Motor motor(FLYWHEEL_PORT);

    


    int targetSpeed = 0;
    double actualSpeed = 0;
    
    auto flywheel_read = okapi::EmaFilter(0.1);
    

    int voltageUpdate() {
        
        
        if (targetSpeed == 0)  {motor.moveVoltage(0); return 0; }
        auto actualSpeed = flywheel_read.filter(motor.getActualVelocity());
        int currentVoltage = actualSpeed * (MAXIMUM_VOLTAGE/TECHNICAL_FLYWHEEL_RPM);
        int convertedTarget = targetSpeed * (MAXIMUM_VOLTAGE/TECHNICAL_FLYWHEEL_RPM);
        int error = (convertedTarget - currentVoltage) * 0.5;
        int finalVoltage = convertedTarget + error;
        motor.moveVoltage(finalVoltage);
        
        return finalVoltage;
    };

    void setTargetSpeed(double pwr) {
        targetSpeed = TECHNICAL_FLYWHEEL_RPM * pwr;
        if (targetSpeed > 0) brake(false);
        voltageUpdate();
    };

}
