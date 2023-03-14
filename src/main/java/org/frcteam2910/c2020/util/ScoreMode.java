package org.frcteam2910.c2020.util;

public enum ScoreMode{
    ZERO(0.001, 0.25), 
    CONE_INTAKE(17, 3.375), //17, 3.375 practice : 15 , 0.1
    MID(87, 0.875),
    HIGH(103, 12.875), // "money" - paul
    LOW(33, 3.375),
    CUBE_INTAKE(33, 9.5); //33, 9.5 practice : 24.5, 4.25

    ScoreMode(double angle, double inches){
        this.angle = angle;
        this.inches = inches;
    }

    public static ScoreMode getClosestMode(double angle){
        if(angle<=-95){
            return HIGH;
        }
        else if(angle<=-50){
            return MID;
        }
        else if(angle<=-15){
            return LOW;
        }
        else if(angle<=15 && angle >= -15){
            return ZERO;
        }
        else if(angle > 23){
            return CUBE_INTAKE;
        }
        else{
            return CONE_INTAKE;
        }
    }

    public double getAngle(){
        return getAngle(false);
    }

    public double getAngle(boolean negative){
        return (negative?-1:1)*angle;
    }

    public double getAutonAngle(){
        if(this == CUBE_INTAKE || this == CONE_INTAKE){
            return angle;
        }
        else{
            return -1*angle;
        }
    }

    public double getInches(){
        return inches;
    }

    double angle;
    double inches;
}