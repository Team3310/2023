package org.frcteam2910.c2020.util;

public enum ScoreMode{
    ZERO(0, 0), 
    CONE_INTAKE(17, 3.375),
    MID(87, 0.875),
    HIGH(108, 12.875),
    LOW(33, 3.375),
    CUBE_INTAKE(33, 9.5);

    ScoreMode(double angle, double inches){
        this.angle = angle;
        this.inches = inches;
    }

    public double getAngle(){
        return angle;
    }

    public double getInches(){
        return inches;
    }

    double angle;
    double inches;
}