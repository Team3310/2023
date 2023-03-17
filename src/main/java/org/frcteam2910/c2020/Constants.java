package org.frcteam2910.c2020;

public class Constants {

    //#region DrivetrainSubsystem
    /*
    Wheel gears to the left for offsets
     */

    public static final double DRIVETRAIN_VOLTAGE_RAMP = 5.0;

    public static final double ROTATIONAL_SCALAR = 0.8;
    public static final double TRANSLATIONAL_SCALAR = 0.6;

    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 9;
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 3;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(347.780); //347.78 Practice robot settings
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_COMP_OFFSET = -Math.toRadians(-0.53); //3.947 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 11;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 10;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 2;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(35.77); //35.77 Practice robot settings
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_COMP_OFFSET = -Math.toRadians(21.71+180); //259.89 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 1;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(169.01); //169.01 Practice robot settings
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_COMP_OFFSET = -Math.toRadians(262.35);//168.76 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 19;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 18;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 0;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(297.86-180); //297.86 Practice robot settings
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_COMP_OFFSET = -Math.toRadians(204.17); //204.52 Comp settings
    /********************************************************************************************************************/

    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;

    public static final int PIGEON_PORT = 0;

    
    public static final double BALANCE_DEADBAND = 0.5;
    public static final double DRIVE_ROTATION_JOYSTICK_DEADBAND = 0.2;
    //#endregion
    
    //#region Intake
    // public static final double SERVO_OUT_DEGREES = 135.0;
    // public static final double SERVO_IN_DEGREES = 0.0;
    public static final int RIGHT_SERVO_PORT = 0;
    public static final int LEFT_SERVO_PORT = 1;
    public static final int ARM_ROTATION_MOTOR_PORT = 14;
    public static final int ARM_TRANSLATIONAL_MOTOR_PORT = 13;
    public static final int INTAKE_MOTOR_PORT = 5;
    
    // Arm Home Values, Offsets, and Ratios
                                                        // (50.0/11.0)*(60.0/26.0)*(36.0/18.0)*(36.0/18.0)*(72.0/18.0);
    public static final double ARM_ROTATION_GEAR_RATIO = (50.0/11.0)*
                                                         (60.0/26.0)*
                                                         (36.0/18.0)*
                                                         (36.0/18.0)*
                                                         (72.0/18.0);
    public static final double ARM_REVOLUTIONS_TO_ENCODER_TICKS = ARM_ROTATION_GEAR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    public static final double ARM_DEGREES_TO_ENCODER_TICKS = ARM_REVOLUTIONS_TO_ENCODER_TICKS/360.0;
    public static final double ARM_HOME_DEGREES = 0.0;

    public static final double ARM_TRANSLATIONAL_GEAR_RATIO = (36/12.0)*(36/18.0);
    public static final double ARM_EXTEND_ZEROING_SPEED = 0.25;
    public static final double ARM_EXTEND_HOME_INCHES = 0.0;
    
    // Define Arm motion parameters
    public static final double ARM_MIN_ROTATION_DEGREES = -110.0;
    public static final double ARM_MAX_ROTATION_DEGREES = 110.0;
    
    public static final double ARM_MIN_EXTEND_INCHES = 0.0;
    public static final double ARM_MAX_EXTEND_INCHES = 15.0;

    // Intake Motor Constants
    public static final double INTAKE_COLLECT_RPM = 1500.0;
    public static final double INTAKE_SPIT_RPM = -1300.0;

    public static final double INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO = 50.0 / 12.0;
    public static final double INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS = INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO * ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    //#endregion
}
