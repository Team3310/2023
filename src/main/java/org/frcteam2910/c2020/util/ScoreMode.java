package org.frcteam2910.c2020.util;

public enum ScoreMode{
    ZERO(0, 0), 
    HIGH(-108, 12.875), 
    MID(-87, 0.875), 
    INTAKE(22, 4.0);

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