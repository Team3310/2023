package org.frcteam2910.c2020.util;

public enum ScoreMode{
    ZERO(0, 0), 
    HIGH(-108, 12), 
    MID(-87, 0), 
    INTAKE(22, 4);

    ScoreMode(double angle, double inches){
        this.angle = angle;
        this.inches = inches;
    }

    double angle;
    double inches;
}