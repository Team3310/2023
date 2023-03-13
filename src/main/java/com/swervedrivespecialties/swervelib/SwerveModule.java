package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public interface SwerveModule {
    double getDriveVelocity();

    double getTargetDriveVoltage();

    double getSteerAngle();

    double getTargetSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void setVoltageRamp(double rampTime);

    void resetAbsoluteSteerAngle();

    void setDriveNeutralMode(NeutralMode neutralMode);

    void setSteerNeutralMode(NeutralMode neutralMode);

    void setEncoderAutoResetIterations(int iterations);
}
