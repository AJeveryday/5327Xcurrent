#include "main.h"




#define FLYWHEEL_GEARSET pros::E_MOTOR_GEARSET_06
#define INTAKE_GEARSET pros::E_MOTOR_GEARSET_06
namespace flywheel{
     pros::Motor motor(FLYWHEEL_PORT, FLYWHEEL_GEARSET);
    
    int targetSpeed = 0;
    double actualSpeed = 0;

    int voltageUpdate() {
        if (targetSpeed == 0)  {motor.move_voltage(0); return 0; }
        actualSpeed = motor.get_actual_velocity();
        int currentVoltage = actualSpeed * (MAXIMUM_VOLTAGE/TECHNICAL_FLYWHEEL_RPM);
        int convertedTarget = targetSpeed * (MAXIMUM_VOLTAGE/TECHNICAL_FLYWHEEL_RPM);
        int error = (convertedTarget - currentVoltage) * 0.5;
        int finalVoltage = convertedTarget + error;
        motor.move_voltage(finalVoltage);
        return finalVoltage;
    };

    void setTargetSpeed(double pwr) {
        targetSpeed = TECHNICAL_FLYWHEEL_RPM * pwr;
        if (targetSpeed > 0) brake(false);
        voltageUpdate();
    };
    void brake(bool mode) {
        if (mode == true && abs(actualSpeed) < 30) motor.set_brake_mode(MOTOR_BRAKE_BRAKE);
        else motor.set_brake_mode(MOTOR_BRAKE_COAST);
    };
}