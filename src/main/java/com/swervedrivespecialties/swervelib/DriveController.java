package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getReferenceVoltage();

    void setMotorNeutralMode(NeutralMode neutralMode);

    void setVoltageRamp(double rampTime);
}
