package org.frcteam2910.c2020.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Pigeon;
import org.frcteam2910.c2020.Robot;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.util.SideChooser.SideMode;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.PlaystationController;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.BallColor;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;


public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {

    //////////////////////////////////////////////////////////////
    //                                                          //
    //                   Class Data and States                  //
    //                                                          //
    //////////////////////////////////////////////////////////////

    private static DrivetrainSubsystem INSTANCE;

    private static final double TRACKWIDTH = 0.502;
    private static final double WHEELBASE = 0.502;
    private static final double WHEEL_DIAMETER_INCHES = 4.00;  // Actual is 3.89"

    private double frontLeftOffset = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_COMP_OFFSET;
    private double frontRightOffset = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_COMP_OFFSET;
    private double backLeftOffset = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_COMP_OFFSET;
    private double backRightOffset = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_COMP_OFFSET;

    private SideMode side = SideMode.BLUE;
    private double rCurrPoseX;
    private double rCurrPoseY;
    private boolean isRight = true;
    private double targetAngle; // for limelight and probably ball tracking
    private boolean isLimelightOverride = false;

    private Vector2 balanceInitialPos = Vector2.ZERO;
    private Timer balanceTimer = new Timer();
    
    // Initialized in constructor
    private ShuffleboardTab tab;
    // #region --Motors and Sensors--

    private final SwerveModule[] modules;
    private double[] lastModuleAngle;

    Limelight limelightGoal = Limelight.getInstance();
    // Must be named limelight-ball in the Limelight config
    Limelight limelightBall = new Limelight();

    private final Object sensorLock = new Object();
    @GuardedBy("sensorLock")
    private final Pigeon gyroscope = new Pigeon(Constants.PIGEON_PORT);
    //#endregion

    // #region --Driving and Kinematics--
    private Controller primaryController;  // For the driver
    private DriveControlMode driveControlMode = DriveControlMode.JOYSTICKS;
    private SwervePivotPoint pivotPoint = SwervePivotPoint.CENTER;

    // Determines abilities of controller
    public enum  DriveControlMode
    {
        JOYSTICKS,
        ROBOT_CENTRIC,
        TURN_TO_GOAL,
        BALL_TRACK,
        LIMELIGHT,
        LIMELIGHT_SEARCH,
        LIMELIGHT_LOCKED,
        TRAJECTORY,
        HOLD,
        LIMELIGHT_BROKEN,
        LIMELIGHT_PROFILED,
        BALANCE, 
        ZERO,
        ;
    }

    public enum SwervePivotPoint
    {
        CENTER,
        BACK
    }

    private final SwerveKinematics swerveKinematicsCenter = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),         //front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),        //front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),       //back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)        //back right
    );

    private final SwerveKinematics swerveKinematicsBack = new SwerveKinematics(
            new Vector2(TRACKWIDTH * 1.354 / 2.0, WHEELBASE / 2.0),         //front left
            new Vector2(TRACKWIDTH * 1.354 / 2.0, -WHEELBASE / 2.0),        //front right
            new Vector2(TRACKWIDTH * 0.354 / 2.0, WHEELBASE / 2.0),       //back left
            new Vector2(TRACKWIDTH * 0.354 / 2.0, -WHEELBASE / 2.0)        //back right
    );

    private SwerveKinematics swerveKinematics = swerveKinematicsCenter;

    /*private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(-TRACKWIDTH*0.25, WHEELBASE / 2.0),         //front left
            new Vector2(-TRACKWIDTH*0.25, -WHEELBASE / 2.0),        //front right
            new Vector2(-TRACKWIDTH*1.25, WHEELBASE / 2.0),       //back left
            new Vector2(-TRACKWIDTH*1.25, -WHEELBASE / 2.0)        //back right
    ); */

    /** Locks to do with certain censors or data */
    private final Object stateLock = new Object();

    /** Our main drive signal (Translation, Rotation, FieldOrRobotCentric).
     *  It is set when we tell the chassis to drive (via a command, joystick, or the drive() method).
     *  @see DrivetrainSubsystem.update(double time, double dt)
     * */
    @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new Vector2(0, 0), 0, true);

    private final Object kinematicsLock = new Object();
    
    @GuardedBy("kinematicsLock")
    private Vector2 velocity = Vector2.ZERO;
    @GuardedBy("kinematicsLock")
    private double angularVelocity = 0.0;
    @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;
    @GuardedBy("kinematicsLock")
    private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();
    @GuardedBy("kinematicsLock")
    private final SwerveOdometry swerveOdometry = new SwerveOdometry(RigidTransform2.ZERO);
    
    //#endregion

    //#region Drivetrain PID, Constraints, and Control Systems
    
    public TrapezoidProfile.Constraints constraints = new Constraints(20.0, 6.0);

    //private PidConstants RotationConstants = new PidConstants(0.0, 0.0, 0.0);
    //public PidController limelightController = new PidController(RotationConstants);

    public ProfiledPIDController rotationController = new ProfiledPIDController(2.0, 0.03, 0.02, constraints, 0.02);
    public ProfiledPIDController profiledLimelightController = new ProfiledPIDController(1.0, 0.03, 0.02, constraints, 0.02);
    public PIDController limelightController = new PIDController(2.0, 0.03, 0.25, 0.02); //(3.0, 0.03, 0.02) (1.7, 0.03, 0.25) 0.02
    public PIDController ballTrackController = new PIDController(1.0, 0.03, 0.25, 0.02);
    private PidController balanceController = new PidController(new PidConstants(0.5, 0.0, 0.02));

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = 
        new DrivetrainFeedforwardConstants(
            0.042746,
            0.0032181,
            0.30764
        );

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = 
    {
        new FeedforwardConstraint(11.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(), FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
        new MaxAccelerationConstraint(12.5 * 12.0),
        new CentripetalAccelerationConstraint(15 * 12.0)
    };

    private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

    private final HolonomicMotionProfiledTrajectoryFollower follower =
        new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(0.4, 0.0, 0.025),
            new PidConstants(5.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS)
        );

    // Logging of odometry to SmartDashboard/Shuffleboard
    private final GenericEntry odometryXEntry;
    private final GenericEntry odometryYEntry;
    private final GenericEntry odometryAngleEntry;
    // #endregion

    //#region Constructor/Class Methods

    public static DrivetrainSubsystem getInstance() {
        if(INSTANCE == null) {
            INSTANCE = new DrivetrainSubsystem();
        }
        return INSTANCE;
    }

    private DrivetrainSubsystem() {  
        synchronized (sensorLock) {
            gyroscope.setInverted(false);
        }

        if(Robot.isPracticeBot()){
            frontLeftOffset = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET;
            frontRightOffset = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET;
            backLeftOffset = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET;
            backRightOffset = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET;
        }

        modules = new SwerveModule[4];
        lastModuleAngle = new double[4];        
        tab = Shuffleboard.getTab("Drivetrain");

        createSwerveModulesAndShuffleboardInfo();
        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 3)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(1, 3)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(2, 3)
                .withSize(1, 1)
                .getEntry();

        for(SwerveModule i : modules){
            i.setVoltageRamp(0.45);
        }

        limelightController.setTolerance(0.1);
        limelightController.setIntegratorRange(-1.0, 1.0);

        // TODO change these pi values
        balanceController.setInputRange(0, Math.PI);
        balanceController.setContinuous(true);
    }
    //#endregion

    //#region Class Methods

    /** Lays out the Shuffleboard format out-of-the-box for this Drivetrain configuration. 
     * 
     * Check the 'see's for where the motors must connect to the CANivore CAN bus with "Drivetrain" as its CAN group.
     * @see SwerveDriveSpecialties Falcon500Drive(and Steer)ControllerFactory
    */
    private void createSwerveModulesAndShuffleboardInfo() {
        //CANivore string key is "Drivetrain" which can be located in the ctre Falcon 500 factories
        SwerveModule frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withPosition(0, 0)
                        .withSize(1, 3),
                Mk4SwerveModuleHelper.GearRatio.L2i,
                Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                frontLeftOffset
        );
        SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withPosition(1, 0)
                        .withSize(1, 3),
                Mk4SwerveModuleHelper.GearRatio.L2i,
                Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                frontRightOffset
        );
        SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withPosition(2, 0)
                        .withSize(1, 3),
                Mk4SwerveModuleHelper.GearRatio.L2i,
                Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                backLeftOffset
        );
        SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withPosition(3, 0)
                        .withSize(1, 3),
                Mk4SwerveModuleHelper.GearRatio.L2i,
                Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                backRightOffset
        );

        modules[0] = frontLeftModule;
        modules[1] = frontRightModule;
        modules[2] = backLeftModule;
        modules[3] = backRightModule;

        tab.addNumber("Trajectory X", () -> {
                    if (follower.getLastState() == null) {
                        return 0.0;
                    }
                    return follower.getLastState().getPathState().getPosition().x;
                })
                .withPosition(0, 4)
                .withSize(1, 1);
        tab.addNumber("Trajectory Y", () -> {
                    if (follower.getLastState() == null) {
                        return 0.0;
                    }
                    return follower.getLastState().getPathState().getPosition().y;
                })
                .withPosition(1, 4)
                .withSize(1, 1);

        tab.addNumber("Rotation Voltage", () -> {
                HolonomicDriveSignal signal;
                synchronized (stateLock) {
                    signal = driveSignal;
                }

                if (signal == null) {
                    return 0.0;
                }

                return signal.getRotation() * RobotController.getBatteryVoltage();
            }).withPosition(2, 4)
              .withSize(1, 1);

              
        tab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity);
    }
    //#endregion

    //#region Getters and Setters

    public void setController(XboxController controller){
        primaryController = controller;
    }
    public void setController(PlaystationController controller){
        primaryController = controller;
    }

    public void setDriveControlMode(DriveControlMode mode){
        driveControlMode = mode;
    }

    public void setSwervePivotPoint(SwervePivotPoint pivotPoint){
        synchronized (kinematicsLock) {
            if (pivotPoint == SwervePivotPoint.CENTER) {
                swerveKinematics = swerveKinematicsCenter;
            } else if (pivotPoint == SwervePivotPoint.BACK) {
                swerveKinematics = swerveKinematicsBack;
            }
            this.pivotPoint = pivotPoint;
        }
    }

    public void setRotationRight(boolean isRight){
        this.isRight = isRight;
    }


    public DriveControlMode getDriveControlMode(){
        return driveControlMode;
    }

    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    public void setLimelightOverride(boolean isOveridden){
        isLimelightOverride = isOveridden;
        if(isOveridden) {
            limelightGoal.setLedMode(Limelight.LedMode.OFF);
        }
        else{
            limelightGoal.setLedMode(Limelight.LedMode.ON);
        }
    }

    public boolean getLimelightOverride(){
        return isLimelightOverride;
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public Vector2 getVelocity() {
        synchronized (kinematicsLock) {
            return velocity;
        }
    }

    public double getAngularVelocity() {
        synchronized (kinematicsLock) {
            return angularVelocity;
        }
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }
    //#endregion

    //#region Class Methods and Drive Control Modes

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    public void joystickDrive(){
        // DriveControlMode is JOYSTICKS

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        // Set the drive signal to a field-centric (last boolean parameter is true) joystick-based input.
        drive(new Vector2(getDriveForwardAxis().get(true)*Constants.TRANSLATIONAL_SCALAR, getDriveStrafeAxis().get(true)*Constants.ROTATIONAL_SCALAR), getDriveRotationAxis().get(true) * 0.80, true);  
    }

    public void rotationDrive(){
        // DriveControlMode is ROTATION

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = rotationController.calculate(getPose().rotation.toRadians());

        // Set the drive signal to a robot-centric joystick-based input for drive & strafe only (not rotation/chassis angle).
        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);

        double target = getPoseAtTime(Timer.getFPGATimestamp() - limelightGoal.getPipelineLatency() / 1000.0).rotation.toRadians() - Math.toRadians(limelightGoal.getFilteredTargetHorizOffset());

        //System.out.println("Goal = " + Math.toDegrees(rotationController.getGoal().position) +", Target = " + Math.toDegrees(target) + ", Gyro Angle = " + getPose().rotation.toDegrees() + ", Position Error" + Math.abs(rotationController.getPositionError()) + ", Control Mode = " + driveControlMode);

        double error = Math.toDegrees(rotationController.getGoal().position) - getPose().rotation.toDegrees();
        if(Math.abs(error) < 3.0 && limelightGoal.hasTarget()) {
            setDriveControlMode(DriveControlMode.LIMELIGHT);
            //System.out.println("Position Error" + Math.abs(rotationController.getPositionError()) + ", Control Mode = " + driveControlMode);
        }
    }

    public void robotCentricDrive(){
        // DriveControlMode is ROBOT_CENTRIC

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        // Set the drive signal to a robot-centric (last boolean parameter is false) joystick-based input.
        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), getDriveRotationAxis().get(true), false);
    }

    public void ballTrackDrive(){
        // DriveControlMode is BALL_TRACK


        // Pipeline in the Limelight's web interface is set to 0 or 1. 0 is for Red balls, 1 for Blue balls.
        limelightBall.setPipeline(RobotContainer.getInstance().getDriverReadout().getTrackedBallColor() == BallColor.BLUE ? 1 : 0);

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        
        // Set the drive signal to a field-centric or robot-centric joystick-based input when we see a ball.
        if(limelightBall.hasTarget()) {
            ballTrackController.setSetpoint(Math.toRadians(-limelightBall.getFilteredTargetHorizOffset()) + getPose().rotation.toRadians());

            double rotationOutput = ballTrackController.calculate(getPose().rotation.toRadians());

            // Last boolean in drive() is true for field-oriented or false for robot-centric left joystick
            drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
            primaryController.setRumble(RumbleType.kLeftRumble, 0.5);
            primaryController.setRumble(RumbleType.kRightRumble, 0.5);
        }
        // Set the drive signal to a field-centric joystick-based input when we don't see a ball.
        // Also allow rotation from the right joystick on the driver controller.
        else {
            drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), getDriveRotationAxis().get(true), true);
            primaryController.setRumble(RumbleType.kLeftRumble, 0.0);
            primaryController.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }

    public void limelightSearch(){
        // DriveControlMode is LIMELIGHT_SEARCH

        // Chassis must rotate to 'scan' for a limelight (goal) target
        if(!limelightGoal.hasTarget()){

            primaryController.getLeftXAxis().setInverted(true);
            primaryController.getRightXAxis().setInverted(true);

            double rotationOutput = 0.8;

            if(isRight){
                rotationOutput *= -1.0;
            }

            drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
        }
        else {
            setDriveControlMode(DriveControlMode.LIMELIGHT);
        }
    }

    public void balanceOutDrive() {
        // DriveControlMode is BALANCE
        balanceController.reset();
        balanceController.setSetpoint(0);
        //TODO change to field oriented
        double angle = getPose().rotation.toDegrees();
        boolean roll = (angle>60 && angle<120)||(angle>240 && angle<300);
        boolean pitch = roll?false:(angle>330 || angle<30)||(angle>150 && angle<210);
        boolean tiltedBackward = false;
        double invertOutput = 1.0;
        double degreesAwayFromBalance = 0;

        SmartDashboard.putBoolean("isRoll", roll);
        SmartDashboard.putBoolean("isPitch", pitch);
        //if(degreesAwayFromBalance>1)
        //    degreesAwayFromBalance*=degreesAwayFromBalance;
        if(roll){
            degreesAwayFromBalance = getRollDegreesOffLevel();
        }else if(pitch){
            degreesAwayFromBalance = getPitchDegreesOffLevel();
        }else{
            degreesAwayFromBalance = 0.0; 
            setDriveControlMode(DriveControlMode.JOYSTICKS);
            return;       
        }

        SmartDashboard.putNumber("degrees away", degreesAwayFromBalance);

        double x = getPose().translation.x;

        if(side==SideMode.RED){
            if(x<-210){
                invertOutput = -1.0;
            }
            else if(x>-210){
                invertOutput = 1.0;
            }
        }
        else if(side==SideMode.BLUE){    
            if(x>210){
                invertOutput = -1.0;
            }
            else if(x<210){
                invertOutput = 1.0;
            }
        }

        double forwardAxisOutput = balanceController.calculate(Math.toRadians(degreesAwayFromBalance), 0.02);
        
        if(degreesAwayFromBalance < Constants.BALANCE_DEADBAND)
        {
            forwardAxisOutput *= 1.2;
            if(!balanceTimer.hasElapsed(0.1))
                balanceTimer.start();
        }
        else if(degreesAwayFromBalance>10)
        {
            balanceTimer.stop();
            balanceTimer.reset();
            forwardAxisOutput *= 1.5;
        }
        else
        {
            balanceTimer.stop();
            balanceTimer.reset();
            forwardAxisOutput *= .8;
        }

        SmartDashboard.putNumber("invert", invertOutput);
        SmartDashboard.putNumber("output", forwardAxisOutput);

        drive(new Vector2(invertOutput * forwardAxisOutput, 0.0), 0.0, true);
    }

    public void limelightDrive(){
        // Config config = new Config(152.4/1000, angularVelocity, angularVelocity, angularVelocity, angularVelocity);
        // AprilTagPoseEstimator poseEstimator = new AprilTagPoseEstimator(config);
        // AprilTagDetection aprilTagDetection = new AprilTagDetection(limelightBall., MAX_LATENCY_COMPENSATION_MAP_ENTRIES, MAX_LATENCY_COMPENSATION_MAP_ENTRIES, MAX_LATENCY_COMPENSATION_MAP_ENTRIES, lastModuleAngle, TRACKWIDTH, MAX_LATENCY_COMPENSATION_MAP_ENTRIES, lastModuleAngle)
        // AprilTagDetector aprilTagDetector = new AprilTagDetector();
        // poseEstimator.
        // DriveControlMode is LIMELIGHT
        targetAngle = getPoseAtTime(Timer.getFPGATimestamp() - limelightGoal.getPipelineLatency() / 1000.0).rotation.toRadians() - Math.toRadians(limelightGoal.getFilteredTargetHorizOffset());

        //targetAngle = Math.toRadians(-limelightGoal.getFilteredTargetHorizOffset()) + getPose().rotation.toRadians();

        limelightController.setSetpoint(targetAngle);

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = limelightController.calculate(getPose().rotation.toRadians());

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
    }

    public void limelightProfiledDrive(){
        // DriveControlMode is LIMELIGHT

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = profiledLimelightController.calculate(getPose().rotation.toRadians());

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);

        if(Math.toDegrees(profiledLimelightController.getPositionError()) < 0.05 && limelightGoal.hasTarget()) {
            setDriveControlMode(DriveControlMode.LIMELIGHT);
        }
    }

    public void limelightLockedDrive(){
        // DriveControlMode is LIMELIGHT_LOCKED

        limelightController.setSetpoint(Math.toRadians(-limelightGoal.getFilteredTargetHorizOffset()) + getPose().rotation.toRadians());

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = limelightController.calculate(getPose().rotation.toRadians());

        // No x-y plane movement allowed, and we tell the PIDController to snap to the calculated angle
        drive(new Vector2(0, 0), rotationOutput, true);
    }
    //#endregion

    //#region Utility (Control System) Methods and Calculations

    public void resetSteerAbsoluteAngle() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].resetAbsoluteSteerAngle();
        }
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }

    public void alignWheels() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].set(0,0);
        }
    }

    public void setDriveCoast(){
        for(SwerveModule i : modules){
            i.setDriveNeutralMode(NeutralMode.Coast);
        }
    }

    public void setDriveBrake(){
        for(SwerveModule i : modules){
            i.setDriveNeutralMode(NeutralMode.Brake);
        }
    }

    public void setSteerCoast(){
        for(SwerveModule i : modules){
            i.setSteerNeutralMode(NeutralMode.Coast);
        }
    }

    public void setSide(SideMode side){
        this.side = side;
    }

    public void setSteerBrake(){
        for(SwerveModule i : modules){
            i.setSteerNeutralMode(NeutralMode.Brake);
        }
    }

    public void setSteerEncoderAutoResetIterations(int iterations) {
        // 500 iterations is the default, -1 turns off auto reset
        for (int i = 0; i < modules.length; i++) {
            modules[i].setEncoderAutoResetIterations(iterations);
        }
    }

    public RigidTransform2 getPoseAtTime(double timestamp) {
        synchronized (kinematicsLock) {
            if (latencyCompensationMap.isEmpty()) {
                return RigidTransform2.ZERO;
            }
            return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
        }
    }

    public void setRotationTarget(double goal){
        rotationController.enableContinuousInput(0.0, Math.PI*2);
        rotationController.reset(getPose().rotation.toRadians());
        rotationController.setGoal(goal + getPose().rotation.toRadians());
        rotationController.setTolerance(0.017);
        setDriveControlMode(DriveControlMode.TURN_TO_GOAL);
    }    
    
    public void setTurnToTarget(){
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.reset(getPose().rotation.toRadians());
        rotationController.setGoal(Math.toRadians(getRobotToGoalAngle()));
        rotationController.setTolerance(0.017);
        setDriveControlMode(DriveControlMode.TURN_TO_GOAL);
    }

    public boolean atRotationTarget(){
        
        if(rotationController.atGoal()){
            //System.out.println("Reached target");
        }
        return rotationController.atGoal();
    }

    public void setBalanceInitialPos(Vector2 initialPos) {
        if(!balanceInitialPos.equals(initialPos, 2)) {
            balanceInitialPos = initialPos;
        }
    }

    public Vector2 getBalanceInitialPos() {
        return balanceInitialPos;
    }

    public Timer getBalanceTimer() {
        return balanceTimer;
    }

    public boolean isBalanced() {
        if(balanceTimer!=null && balanceTimer.hasElapsed(2.5)){
            balanceTimer.stop();
            balanceTimer.reset();
            return true;
        }
        else    
            return false;
    }
    //#endregion

    //#region Calculations

    // public double getLagAngleDegrees(){
    //     double timeOfFlight = Shooter.getInstance().getBallFlightTime();
    //     double distance = Shooter.getInstance().getActualDistanceFromGoal();

    //     return Math.toDegrees(Math.atan((getVelocity().y * timeOfFlight)/distance));
    // }

    public double getRobotToGoalAngle() {
        rCurrPoseX = getPose().translation.x;
        rCurrPoseY = getPose().translation.y;
        double fieldAngle = Math.toDegrees(Math.atan2(rCurrPoseY + 162, rCurrPoseX - 325));

        return fieldAngle;
    }

    public double getRobotToGoalDistance() {
        rCurrPoseX = getPose().translation.x - 325;
        rCurrPoseY = getPose().translation.y + 162;
        return Math.sqrt(rCurrPoseX*rCurrPoseX + rCurrPoseY*rCurrPoseY);
    }

    public double getAverageAbsoluteValueVelocity() {
        double averageVelocity = 0;
        for (var module : modules) {
            averageVelocity += Math.abs(module.getDriveVelocity());
        }
        return averageVelocity / 4;
    }

    /**
     * Must be implemented for <code>UpdateManager</code> (Updatable interface method)
     * @param time - Time in seconds from Timer.getFPGATimeStamp() (robot runtime timer).
     * @param dt - Time since update was last called
     */
    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);

        DriveControlMode i_controlMode = getDriveControlMode();

        HolonomicDriveSignal currentDriveSignal = null;

        //System.out.println("Control Mode: " + driveControlMode);

        switch(i_controlMode){
            case JOYSTICKS:
                joystickDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case LIMELIGHT_BROKEN:
                joystickDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case TURN_TO_GOAL:
                rotationDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case ROBOT_CENTRIC:
                robotCentricDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case BALL_TRACK:
                ballTrackDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case LIMELIGHT:
                limelightDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case LIMELIGHT_LOCKED:
                limelightLockedDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case LIMELIGHT_SEARCH:
                limelightSearch();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case LIMELIGHT_PROFILED:
                limelightProfiledDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case TRAJECTORY:
                Optional<HolonomicDriveSignal> trajectorySignal = follower.update(
                        getPose(),
                        getVelocity(),
                        getAngularVelocity(),
                        time,
                        dt
                );
                if (trajectorySignal.isPresent()) {
                    currentDriveSignal = trajectorySignal.get();
                    currentDriveSignal = new HolonomicDriveSignal(
                        currentDriveSignal.getTranslation().scale(1.0 / RobotController.getBatteryVoltage()),
                        currentDriveSignal.getRotation() / RobotController.getBatteryVoltage(),
                        currentDriveSignal.isFieldOriented()
                    );
                }
                break;
            case BALANCE:
                balanceOutDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case HOLD:
                updateModulesHold();
                break;
            case ZERO:
                setModulesAngle(0.0);
                break;    
        }

        if(driveControlMode != DriveControlMode.BALL_TRACK) {
            // Handle rumble runaway
            primaryController.setRumble(RumbleType.kLeftRumble, 0);
            primaryController.setRumble(RumbleType.kRightRumble, 0);
        }

        if(driveControlMode != DriveControlMode.HOLD) {
            // Tell all swerve modules their new targets based on the kinematics.
            updateModules(currentDriveSignal);
        }
    }

    public double getPitch(){
        return gyroscope.getPitch().toDegrees() - 2; // -2 for bias when level
    }

    public double getRoll(){
        return gyroscope.getRoll().toDegrees() + 0.3; // +0.3 for bias when level
    }
    
    public double getPitchDegreesOffLevel(){
        double pitch = getPitch();
        if(pitch > 180) {
            pitch = 360 - pitch;
        }
        return pitch;
    }

    public double getRollDegreesOffLevel(){
        double roll = getRoll();
        if(roll > 180) {
            roll = 360 - roll;
        }
        return roll;
    }

    private void updateOdometry(double time, double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getSteerAngle())).scale(module.getDriveVelocity() * 39.37008 * WHEEL_DIAMETER_INCHES / 4.0);
        }

        Rotation2 angle;
        double angularVelocity;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
            angularVelocity = gyroscope.getRate();
        }

        //SmartDashboard.putNumber("Current Gyro Angle", angle.toDegrees());
        ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);

        synchronized (kinematicsLock) {
            if(getPitchDegreesOffLevel()<2 && getRollDegreesOffLevel()<2){
                // On the ground, update odometry
                this.pose = swerveOdometry.update(swerveKinematics, angle, dt, moduleVelocities);
            }
            else {
                // Not on the ground
                double xacc = gyroscope.getAccels()[0]/(double)Short.MAX_VALUE;
                double yacc = gyroscope.getAccels()[1]/(double)Short.MAX_VALUE;
                if((xacc < .12 && getPitchDegreesOffLevel()>Constants.BALANCE_DEADBAND+2)
                    || (yacc < .12 && getRollDegreesOffLevel()>Constants.BALANCE_DEADBAND+2)) {
                    Vector2[] zeros = {new Vector2(0,0),new Vector2(0,0),new Vector2(0,0),new Vector2(0,0)};
                    this.pose = swerveOdometry.update(swerveKinematics, angle, dt, zeros);
                }
                else this.pose = swerveOdometry.update(swerveKinematics, angle, dt, moduleVelocities);
            }    
            if (latencyCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                latencyCompensationMap.remove(latencyCompensationMap.firstKey());
            }
            latencyCompensationMap.put(new InterpolatingDouble(time), pose);
            this.velocity = velocity.getTranslationalVelocity();
            this.angularVelocity = angularVelocity;
        }
    }

    /** Updates ChassisVelocity and calculates SwerveModule velocity outputs. */
    private void updateModules(HolonomicDriveSignal driveSignal) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    driveSignal.getRotation()
            );
        } else {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation(),
                    driveSignal.getRotation()
            );
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);


        if(driveSignal != null && driveSignal.getTranslation() != null && driveSignal.getTranslation().length < 0.05 && Math.abs(driveSignal.getRotation()) < 0.05){
            for (int i = 0; i < moduleOutputs.length; i++) {
                var module = modules[i];
                module.set(0.0, lastModuleAngle[i]);
            }
        }
        else{
            for (int i = 0; i < moduleOutputs.length; i++) {
                var module = modules[i];
                module.set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians());
                lastModuleAngle[i] = moduleOutputs[i].getAngle().toRadians();
            }
        }
    }

    private void updateModulesHold() {

        double[] moduleAngles = {45.0, -45.0, -45.0, 45.0};

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.set(0, Math.toRadians(moduleAngles[i]));
        }
    }

    public void setModulesAngle(double angle) {
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.set(0, Math.toRadians(angle));
        }
    }
    //#endregion

    @Override
    public void periodic() {
        // Must update the Tx/Ty filter to provide it samples for calculation
        limelightGoal.updateTxFilter();
        limelightGoal.updateTyFilter();

        limelightBall.updateTxFilter();
        limelightBall.updateTyFilter();

        // Update SmartDashboard/Shuffleboard data
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(pose.rotation.toDegrees());
        SmartDashboard.putNumber("Angle to goal", getRobotToGoalAngle());
        SmartDashboard.putNumber("Distance to goal", getRobotToGoalDistance());
        SmartDashboard.putNumber("X", pose.translation.x);
        SmartDashboard.putNumber("Y", pose.translation.y);
        SmartDashboard.putNumber("pitch", getPitch());
        SmartDashboard.putNumber("roll", getRoll());
        SmartDashboard.putString("Drive Mode", getDriveControlMode().toString());
        SmartDashboard.putNumber("blanace timer length", balanceTimer.get());
        SmartDashboard.putNumber("angle", getPose().rotation.toDegrees());
        SmartDashboard.putString("side", side.toString());
        SmartDashboard.putNumber("x accel", gyroscope.getAccels()[0]/(double)Short.MAX_VALUE);
        SmartDashboard.putNumber("y accel", gyroscope.getAccels()[1]/(double)Short.MAX_VALUE);
        SmartDashboard.putNumber("z accel", gyroscope.getAccels()[2]/(double)Short.MAX_VALUE);
        //SmartDashboard.putString("accel", gyroscope.getAccels()[0]+" "+gyroscope.getAccels()[1]+" "+gyroscope.getAccels()[2]);
    }
}
