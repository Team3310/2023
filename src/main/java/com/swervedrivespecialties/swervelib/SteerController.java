package com.swervedrivespecialties.swervelib;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void resetAbsoluteSteerAngle();

    void setEncoderAutoResetIterations(int iterations);
}
