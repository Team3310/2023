package org.frcteam2910.c2020;

import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class Constants {
    /*
    Wheel gears to the left for offsets
     */

    public static final double DRIVETRAIN_VOLTAGE_RAMP = 5.0;


    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 9;
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 3;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(0); //68.81 Practice robot settings
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_COMP_OFFSET = -Math.toRadians(0); //64.07 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 11;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 10;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 1;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(346.9); //192.33 Practice robot settings
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_COMP_OFFSET = -Math.toRadians(346.9); //152.49 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 2;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(262.44); //314.73 Practice robot settings
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_COMP_OFFSET = -Math.toRadians(262.44);//48.34 Comp settings
    /********************************************************************************************************************/

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 19;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 18;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 0;

    /********************************************************************************************************************/
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(200.21); //192.91 Practice robot settings
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_COMP_OFFSET = -Math.toRadians(200.21); //213.48 Comp settings
    /********************************************************************************************************************/

    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;

    public static final int PIGEON_PORT = 0;

    
    // Intake
    public static final double INTAKE_COLLECT_RPM = 950; // 1500
    public static final double INTAKE_SLOW_RPM = 500;
    public static final double INTAKE_RETRACT_RPM = 1000;
    public static final double INTAKE_COLLECT_AUTO_RPM = 1200; // 1500
    public static final double INTAKE_REVERSE_RPM = -1000; // -1500
    public static final int INTAKE_MOTOR_CAN_ID = 2;
    public static final double SERVO_OUT_DEGREES = 135.0;
    public static final double SERVO_IN_DEGREES = 0.0;

    //Intake Lift
    public static final double LIFT_COMPETITION_HOME_POSITION_DEGREES = 39.0;//39.0;
    public static final int LIFT_MM_PORT = 0;
    public static final int LIFT_PID_PORT = 1;
    public static final double LIFT_MIN_ANGLE_DEGREES = 0.0;
    public static final double LIFT_MAX_ANGLE_DEGREES = 38.0;
    public static final double LIFT_ZERO_SPEED = 0.1;
    public static final int INTAKE_LIFT_MOTOR_CAN_ID = 5;

    // Indexer
    public static final double INDEXER_RPM = 500;
    public static final double AUTON_INDEXER_RPM = 500;
    public static final int INDEXER_DIO_PORT = 0;
    public static final int INDEX_MOTOR_ID = 12;

    //Shooter
    public static final double IDLE_SHOOTER_RPM = 2150.0;
    public static final int SHOOTER_MASTER_CAN_ID = 13;
    public static final int SHOOTER_SLAVE_CAN_ID = 11;

    //Hood
    public static final double HOOD_MIN_ANGLE_DEGREES = 0;
    public static final double HOOD_MAX_ANGLE_DEGREES = 50;
    public static final double HOOD_COMPETITION_HOME_POSITION_DEGREES = 50;
    public static final double HOOD_ZERO_SPEED = 0.1;
    public static final int HOOD_MM_PORT = 0;
    public static final double IDLE_HOOD_ANGLE = 32.0;
    public static final int HOOD_PID_PORT = 1;
    public static final int HOOD_MOTOR_CAN_ID = 10;

    //Climb Elevator
    public static final double ELEVATOR_MAX_INCHES = 57.5;
    public static final double ELEVATOR_MIN_INCHES = -12.0;
    public static final double ELEVATOR_AUTO_ZERO_SPEED = -0.3;
    public static final double ELEVATOR_HOME_POSITION = 0.0;
    public static final double MIN_CLIMB_ELEVATOR_PERCENT_BUS = 0.1;
    public static final int CLIMB_ELEVATOR_MM_PORT = 0;
    public static final int ELEVATOR_MOTOR_MASTER_ID = 3;
    public static final int ELEVATOR_MOTOR_SLAVE_ID = 12;


    //Balance Elevator
    public static final double BALANCE_ELEVATOR_MIN_INCHES = 0;
    public static final double BALANCE_ELEVATOR_MAX_INCHES = 21.4; //18.9;
    public static final int BALANCE_ELEVATOR_MM_PORT = 0;
    public static final int BALANCE_ELEVATOR_PID_PORT = 1;
    public static final double MIN_BALANCE_ELEVATOR_PERCENT_BUS = 0.06;
    public static final int BALANCE_ELEVATOR_MASTER_ID = 4;
    public static final int BALANCE_ELEVATOR_SLAVE_ID = 16;

    // Bridge Balance
    public static final double BALANCE_DEADBAND = 0.5;



    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLobHoodMap = new InterpolatingTreeMap<>();
    static {
        kLobHoodMap.put(new InterpolatingDouble(48.0), new InterpolatingDouble(11.0));
        kLobHoodMap.put(new InterpolatingDouble(84.0), new InterpolatingDouble(22.0));
        kLobHoodMap.put(new InterpolatingDouble(114.0), new InterpolatingDouble(29.0));
        kLobHoodMap.put(new InterpolatingDouble(135.0), new InterpolatingDouble(32.0));
        kLobHoodMap.put(new InterpolatingDouble(184.0), new InterpolatingDouble(38.0));
        kLobHoodMap.put(new InterpolatingDouble(226.0), new InterpolatingDouble(42.0));
        kLobHoodMap.put(new InterpolatingDouble(266.0), new InterpolatingDouble(42.0));
        //kLobHoodMap.put(new InterpolatingDouble(295.0), new InterpolatingDouble(45.0));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLobRPMMap = new InterpolatingTreeMap<>();
    static {
        kLobRPMMap.put(new InterpolatingDouble(48.0), new InterpolatingDouble(1780.0));
        kLobRPMMap.put(new InterpolatingDouble(84.0), new InterpolatingDouble(1800.0));
        kLobRPMMap.put(new InterpolatingDouble(114.0), new InterpolatingDouble(2000.0));
        kLobRPMMap.put(new InterpolatingDouble(135.0), new InterpolatingDouble(2055.0));
        kLobRPMMap.put(new InterpolatingDouble(184.0), new InterpolatingDouble(2550.0));
        kLobRPMMap.put(new InterpolatingDouble(226.0), new InterpolatingDouble(2700.0));
        kLobRPMMap.put(new InterpolatingDouble(266.0), new InterpolatingDouble(3060.0));
        //kLobRPMMap.put(new InterpolatingDouble(295.0), new InterpolatingDouble(2975.0));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLimelightDistanceMap = new InterpolatingTreeMap<>();
    static {
        kLimelightDistanceMap.put(new InterpolatingDouble(48.0), new InterpolatingDouble(74.0));
        kLimelightDistanceMap.put(new InterpolatingDouble(84.0), new InterpolatingDouble(111.0));
        kLimelightDistanceMap.put(new InterpolatingDouble(114.0), new InterpolatingDouble(144.0));
        kLimelightDistanceMap.put(new InterpolatingDouble(135.0), new InterpolatingDouble(169.0));
        kLimelightDistanceMap.put(new InterpolatingDouble(184.0), new InterpolatingDouble(214.0));
        kLimelightDistanceMap.put(new InterpolatingDouble(226.0), new InterpolatingDouble(257.0));
        kLimelightDistanceMap.put(new InterpolatingDouble(266.0), new InterpolatingDouble(297.0));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kTimeOfFlightMap = new InterpolatingTreeMap<>();
    static {
        kTimeOfFlightMap.put(new InterpolatingDouble(48.0), new InterpolatingDouble(0.8));
        kTimeOfFlightMap.put(new InterpolatingDouble(84.0), new InterpolatingDouble(0.8));
        kTimeOfFlightMap.put(new InterpolatingDouble(114.0), new InterpolatingDouble(0.8));
        kTimeOfFlightMap.put(new InterpolatingDouble(135.0), new InterpolatingDouble(0.8));
        kTimeOfFlightMap.put(new InterpolatingDouble(184.0), new InterpolatingDouble(0.8));
        kTimeOfFlightMap.put(new InterpolatingDouble(226.0), new InterpolatingDouble(0.8));
        kTimeOfFlightMap.put(new InterpolatingDouble(266.0), new InterpolatingDouble(0.8));
    }
}
