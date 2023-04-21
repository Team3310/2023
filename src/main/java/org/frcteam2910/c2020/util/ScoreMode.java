package org.frcteam2910.c2020.util;

import org.frcteam2910.c2020.Constants;

public enum ScoreMode{
    HOME(14.0, 0.25, Constants.ARM_DEFAULT_PID_SLOT), 
    CONE_INTAKE(20, 2.6, Constants.ARM_INTAKE_PID_SLOT),
    CUBE_INTAKE(4.0, 3.0, Constants.ARM_INTAKE_PID_SLOT),

    CONE_HIGH(110, 15.5, Constants.ARM_CONE_HIGH_PID_SLOT),
    CONE_MID(90, 0.875, Constants.ARM_CONE_MID_PID_SLOT),
    CONE_LOW(40, 0.1, Constants.ARM_LOW_PID_SLOT),

    CUBE_HIGH(Constants.ARM_ROTATOR_MAX_ROTATION_DEGREES, Constants.ARM_EXTENDER_MAX_EXTEND_INCHES, Constants.ARM_CUBE_HIGH_PID_SLOT),
    CUBE_MID(93, 10.375, Constants.ARM_CUBE_MID_PID_SLOT),
    CUBE_LOW(55, 0.1, Constants.ARM_LOW_PID_SLOT),
    ;

    ScoreMode(double angle, double inches, int slot){
        this.angle = angle;
        this.inches = inches;
        this.slot = slot;
    }

    @Deprecated
    public static ScoreMode getClosestMode(double angle){
        if(angle>=95){
            return CONE_HIGH;
        }
        else if(angle>=50){
            return CONE_MID;
        }
        else if(angle>=15){
            return CONE_LOW;
        }
        else if(angle<=15 && angle >= -15){
            return HOME;
        }
        else if(angle > 23){
            return CUBE_INTAKE;
        }
        else{
            return CONE_INTAKE;
        }
    }

    public int getPIDSlot(){
        return this.slot;
    }

    public double getAngle(){
        return getAngle(false);
    }

    public double getAngle(boolean negative){
        return (negative?-1:1)*angle;
    }

    public double getAutonAngle(boolean isCube){
        if(this == CUBE_INTAKE || this == CONE_INTAKE){
            return angle;
        }
        else{
            if(isCube){
                switch(this){
                    case CONE_HIGH : return 100;
                    case CONE_MID : return 85.5;
                    case CONE_LOW : return 40;
                    default : return this.getAngle();
                }
            }
            else{
                switch(this){
                    case CONE_HIGH : return 100;
                    case CONE_MID : return 80;
                    case CONE_LOW : return 40;
                    default : return this.getAngle();
                }
            }
        }
    }

    public double getInches(){
        return inches;
    }

    double angle;
    double inches;
    int slot;
}