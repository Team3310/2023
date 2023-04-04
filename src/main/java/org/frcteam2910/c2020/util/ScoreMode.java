package org.frcteam2910.c2020.util;

public enum ScoreMode{
    HOME(10.0, 0.25), 
    CONE_INTAKE(15, 0.1), //17, 3.375 practice : 15 , 0.1
    CUBE_INTAKE(6.0, 3.0), //33, 9.5 practice : 24.5, 5.25

    CONE_HIGH(103, 14.5), // "money" - paul
    CONE_MID(85, 0.875),
    CONE_LOW(40, 0.1),

    CUBE_HIGH(110, 18.5),
    CUBE_MID(90, 6.875),
    CUBE_LOW(55, 0.1),
    ;

    ScoreMode(double angle, double inches){
        this.angle = angle;
        this.inches = inches;
    }

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
}