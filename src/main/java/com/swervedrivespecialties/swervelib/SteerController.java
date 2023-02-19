package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void resetAbsoluteSteerAngle();

    void setEncoderAutoResetIterations(int iterations);

    void setMotorNeutralMode(NeutralMode neutralMode);
}
