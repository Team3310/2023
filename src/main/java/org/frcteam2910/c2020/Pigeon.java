package org.frcteam2910.c2020;

import com.ctre.phoenix.sensors.Pigeon2;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.Rotation2;

public class Pigeon extends Gyroscope {
    private final Pigeon2 handle;

    public Pigeon(int id) {
        this.handle = new Pigeon2(id, "Drivetrain");
    }

    @Override
    public void calibrate() {
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromDegrees(handle.getYaw());
    }

    public Rotation2 getPitch(){
        return Rotation2.fromDegrees(handle.getPitch());
    }

    public short[] getAccels(){
        short[] refAccels = new short[] {0,0,0};
        handle.getBiasedAccelerometer(refAccels);
        return refAccels;
    }

    public Rotation2 getRoll(){
        return Rotation2.fromDegrees(handle.getRoll());
    }

    @Override
    public double getUnadjustedRate() {
        return 0.0;
    }
}
