package org.frcteam2910.c2020;

import org.frcteam2910.c2020.subsystems.Intake.IntakeStopMode;

public class Constants
{
    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;
    public static final double INCHES_PER_METER = 39.37008;
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;

    //#region DrivetrainSubsystem
    /*
    Wheel gears to the left for offsets
     */

    public static final double DRIVETRAIN_VOLTAGE_RAMP = 5.0;

    public static final double ROTATIONAL_SCALAR = 0.8;
    public static final double TRANSLATIONAL_SCALAR = 0.6;
    public static final double DRIVE_ROTATION_JOYSTICK_DEADBAND = 0.15;
    public static final double BALANCE_DEADBAND = 0.5;

    public static final int PIGEON_PORT = 0;
    

    /********************************************************************************************************************/

    //FRONT_LEFT
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 9;
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 3;

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(233.77); //347.78 Practice robot settings
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_COMP_OFFSET = -Math.toRadians(261.64); //-0.53 Comp settings

    /********************************************************************************************************************/

    //FRONT_RIGHT
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 11;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 10;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 2;

    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(115.04); //35.77 Practice robot settings
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_COMP_OFFSET = -Math.toRadians(143.25); //201.71 Comp settings

    /********************************************************************************************************************/

    //BACK_LEFT
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 1;

    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(210.76); //169.01 Practice robot settings
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_COMP_OFFSET = -Math.toRadians(244.86);//346.80 Comp settings

    /********************************************************************************************************************/

    //BACK_RIGHT
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 19;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 18;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 0;

    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(9.31); //117.86 Practice robot settings
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_COMP_OFFSET = -Math.toRadians(151.1); //60.98 Comp settings

    /********************************************************************************************************************/

    //#endregion
    
    //#region Intake
    // public static final double SERVO_OUT_DEGREES = 135.0;
    // public static final double SERVO_IN_DEGREES = 0.0;
    public static final int RIGHT_SERVO_PORT = 0;
    public static final int LEFT_SERVO_PORT = 1;
    public static final int ARM_ROTATION_MOTOR_PORT = 14;
    public static final int ARM_TRANSLATIONAL_MOTOR_PORT = 13;
    public static final int INTAKE_MOTOR_PORT = 5;
    public static final int CUBE_INTAKE_ROLLER_MOTOR_PORT = 21;
    public static final int CUBE_INTAKE_DEPLOY_MOTOR_PORT = 20;
    public static final int ARM_EXTERNAL_CANCODER_PORT = 5;
    
    // Arm Home Values, Offsets, and Ratios
    public static final double ARM_ROTATOR_EXTERNAL_ENCODER_RATIO = (72.0/18.0);
    
    public static final double ARM_ROTATOR_INTEGRATED_ENCODER_GEAR_RATIO = (50.0/11.0)*
                                                                           (44.0/42.0)*
                                                                           (36.0/18.0)*
                                                                           (36.0/18.0)*
                                                                           ARM_ROTATOR_EXTERNAL_ENCODER_RATIO;

    public static final double ARM_ONE_REVOLUTION_TO_INTEGRATED_ENCODER_TICKS = ARM_ROTATOR_INTEGRATED_ENCODER_GEAR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    public static final double ARM_ROTATOR_ONE_DEGREE_TO_INTEGRATED_ENCODER_TICKS = ARM_ONE_REVOLUTION_TO_INTEGRATED_ENCODER_TICKS/360.0;

    public static final double ARM_ROTATOR_ONE_REVOLUTION_TO_EXTERNAL_ENCODER_TICKS = ARM_ROTATOR_EXTERNAL_ENCODER_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    public static final double ARM_ROTATOR_ONE_DEGREE_TO_EXTERNAL_ENCODER_TICKS = ARM_ROTATOR_ONE_REVOLUTION_TO_EXTERNAL_ENCODER_TICKS/360.0;

    public static final double ARM_EXTENDER_GEAR_RATIO = (18.0/12.0)*
                                                         (36.0/18.0);
    
    // Define Arm motion parameters
    public static final double ARM_ROTATOR_HOME_DEGREES = 0.0;
    public static final double ARM_ROTATOR_MIN_ROTATION_DEGREES = -5.0;
    public static final double ARM_ROTATOR_MAX_ROTATION_DEGREES = 115.0;
    
    public static final double ARM_EXTENDER_MIN_EXTEND_INCHES = 0.0;
    public static final double ARM_EXTENDER_MAX_EXTEND_INCHES = 20.0;

    public static final double ARM_EXTENDER_ZEROING_SPEED = 0.25;
    public static final double ARM_EXTENDER_HOME_INCHES = 0.0;

    public static final double CUBE_LIFT_ZEROING_SPEED = 0.25;

    public static final int ARM_DEFAULT_PID_SLOT = 0;
    public static final int ARM_INTAKE_PID_SLOT = 0;
    public static final int ARM_LOW_PID_SLOT = 0;
    public static final int ARM_CONE_MID_PID_SLOT = 1;
    public static final int ARM_CONE_HIGH_PID_SLOT = 1;
    public static final int ARM_CUBE_HIGH_PID_SLOT = 2;
    public static final int ARM_CUBE_MID_PID_SLOT = 2;

    // Intake Constants

    // ARM INTAKE
    public static final double ARM_INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO = 50.0 / 11.0;
    public static final double ARM_INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS = ARM_INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO * ENCODER_TICKS_PER_MOTOR_REVOLUTION;

    public static final double ARM_CONE_INTAKE_COLLECT_RPM = 2000.0;
    public static final double ARM_CUBE_INTAKE_COLLECT_RPM = -1000.0;
    public static final double ARM_CONE_INTAKE_SPIT_RPM = -2600.0;
    public static final double ARM_CUBE_INTAKE_SPIT_RPM = 400.0;
    public static final double INTAKE_STOP_RPM_THRESHOLD = 5.0;
    public static final IntakeStopMode INTAKE_DEFAULT_STOP_MODE = IntakeStopMode.POSITION;

    // CUBE INTAKE DEPLOY
    public static final double CUBE_INTAKE_DEPLOY_MOTOR_RATIO = (64.0/11.0)*
                                                                (36.0/18.0);
    public static final double CUBE_INTAKE_DEPLOY_ONE_REVOLUTION_IN_ENCODER_TICKS = CUBE_INTAKE_DEPLOY_MOTOR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    public static final double CUBE_INTAKE_DEPLOY_ONE_DEGREE_IN_ENCODER_TICKS = CUBE_INTAKE_DEPLOY_ONE_REVOLUTION_IN_ENCODER_TICKS/360.0;
    
    public static final double CUBE_INTAKE_DEPLOY_HOME_DEGREES = 0;
    public static final double CUBE_INTAKE_DEPLOY_MAX_DEGREES = 121;

    // CUBE INTAKE ROLLER
    public static final double CUBE_INTAKE_ROLLER_MOTOR_RATIO = (18.0/12.0);

    public static final double CUBE_INTAKE_ROLLER_COLLECT_RPM = 1750;
    public static final double CUBE_INTAKE_ROLLER_SPIT_RPM = -3000;
    public static final double CUBE_INTAKE_ROLLER_HANDOFF_RPM = 800;

    public static final double INTAKE_STOP_DELAY = 0.75;
    //#endregion
}
