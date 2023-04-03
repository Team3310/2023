package org.frcteam2910.c2020;

import com.ctre.phoenix.sensors.Pigeon2;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.Rotation2;

public class Pigeon extends Gyroscope {
    private final Pigeon2 handle;
    private double XAxisGyroError=0;
    private double YAxisGyroError=0;

    public Pigeon(int id) {
        this.handle = new Pigeon2(id, "Drivetrain");
    }

    @Override
    public void calibrate() {
    }

    public void zeroGyro(){
        XAxisGyroError = 0;
        YAxisGyroError = 0;
        XAxisGyroError = handle.getRoll();
        YAxisGyroError = handle.getPitch();
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromDegrees(handle.getYaw());
    }

    public double getPitch(){
        return Rotation2.fromDegrees(handle.getPitch()).toDegrees()-YAxisGyroError;
    }

    public double[] getGravityVector(){
        double[] vectors = new double[3];
        handle.getGravityVector(vectors);
        return vectors;
    }

    public short[] getAccels(){
        short[] refAccels = new short[] {0,0,0};
        handle.getBiasedAccelerometer(refAccels);
        return refAccels;
    }

    public double getRoll(){
        return Rotation2.fromDegrees(handle.getRoll()).toDegrees()-XAxisGyroError;
    }

    @Override
    public double getUnadjustedRate() {
        return 0.0;
    }
}
