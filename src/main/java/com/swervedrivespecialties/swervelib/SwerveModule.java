package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void setVoltageRamp(double rampTime);

    void resetAbsoluteSteerAngle();

    void setMotorNeutralMode(NeutralMode neutralMode);

    void setEncoderAutoResetIterations(int iterations);
}
