package org.frcteam2910.c2020;

public class Constants {
    /*
    Wheel gears to the left for offsets
     */

    //#region Drivetrain
    public static final double DRIVETRAIN_VOLTAGE_RAMP = 5.0;

    public static final double ROTATIONAL_SCALAR = 0.8;
    public static final double TRANSLATIONAL_SCALAR = 0.8;

    public static final double INCHES_PER_ONE_METER = 39.37008;

    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 9;
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 3;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(4.53); //68.81 Practice robot settings
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_COMP_OFFSET = -Math.toRadians(4.52); //64.07 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 11;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 10;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 1;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(348.75); //192.33 Practice robot settings
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_COMP_OFFSET = -Math.toRadians(348.75); //152.49 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 2;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(262.71); //314.73 Practice robot settings
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_COMP_OFFSET = -Math.toRadians(262.71);//48.34 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 19;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 18;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 0;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(205.56); //192.91 Practice robot settings
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_COMP_OFFSET = -Math.toRadians(205.56); //213.48 Comp settings
    /********************************************************************************************************************/

    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;

    public static final int PIGEON_PORT = 0;
    //#endregion
    
    //#region Intake
    // public static final double SERVO_OUT_DEGREES = 135.0;
    // public static final double SERVO_IN_DEGREES = 0.0;
    public static final int RIGHT_SERVO_PORT = 0;
    public static final int LEFT_SERVO_PORT = 1;
    public static final int ARM_ROTATION_MOTOR_PORT = 14;
    public static final int ARM_TRANSLATIONAL_MOTOR_PORT = 13;
    public static final int INTAKE_MOTOR_PORT = 2;
    public static final double ARM_ROTATION_GEAR_RATIO = (50/11)*(60/26)*(36/18)*(36/18)*(60/18);
    public static final double ARM_TRANSLATIONAL_GEAR_RATIO = (60/11);
    public static final double MIN_ARM_DEGREES = -100.0;
    public static final double MAX_ARM_DEGREES = 100.0;
    public static final double MIN_ARM_INCHES = 0.0;
    public static final double MAX_ARM_INCHES = 17.0;
    public static final double ARM_HOME_DEGREES = 0.0;
    public static final double ARM_HOME_INCHES = 0.0;
    public static final double INTAKE_COLLECT_RPM = 950.0;
    public static final double INTAKE_SPIT_RPM = -300.0;
    //#endregion
    

    // Bridge Balance
    public static final double BALANCE_DEADBAND = 0.5;

    public static final double DEADBAND_ROTATION_JOYSTICK = 0.2;
}
