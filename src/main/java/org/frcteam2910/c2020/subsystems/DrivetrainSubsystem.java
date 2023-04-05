package org.frcteam2910.c2020.subsystems;

import java.util.Optional;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Pigeon;
import org.frcteam2910.c2020.Robot;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.FeedforwardConstraint;
import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.control.TrajectoryConstraint;
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
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {

    //////////////////////////////////////////////////////////////
    //                                                          //
    //                   Class Data and States                  //
    //                                                          //
    //////////////////////////////////////////////////////////////

    private static DrivetrainSubsystem INSTANCE;

    private static final double TRACKWIDTH = 0.59;
    private static final double WHEELBASE = 0.514;
    private static final double WHEEL_DIAMETER_INCHES = 4.00;  // Actual is 3.89"

    private double frontLeftOffset = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_COMP_OFFSET;
    private double frontRightOffset = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_COMP_OFFSET;
    private double backLeftOffset = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_COMP_OFFSET;
    private double backRightOffset = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_COMP_OFFSET;

    private double rCurrPoseX;
    private double rCurrPoseY;
    private boolean isLimelightOverride = false;
    private double commandedPoseAngleDeg = 0.0;
    private boolean wasJustTurning = false;
    private boolean turbo = false;
    private double voltageOutput;
    private double voltageSteerAngle;
    private double startDegrees;
    private boolean slowBalance;

    private Vector2 balanceInitialPos = Vector2.ZERO;
    private Timer balanceTimer = new Timer();
    
    // Initialized in constructor
    private ShuffleboardTab tab;
    // #region --Motors and Sensors--

    private final SwerveModule[] modules;
    private double[] lastModuleAngle;

    Limelight limelightForward = Limelight.getInstance();
    // Must be named limelight-back in the Limelight config
    Limelight limelightBack = new Limelight("back");

    private final Object sensorLock = new Object();
    @GuardedBy("sensorLock")
    private final Pigeon gyroscope = new Pigeon(Constants.PIGEON_PORT);
    //#endregion

    // #region --Driving and Kinematics--
    private Controller primaryController;  // For the driver
    private DriveControlMode driveControlMode = DriveControlMode.JOYSTICKS;
    private LimelightMode limelightMode = LimelightMode.NONE;
    private SwervePivotPoint pivotPoint = SwervePivotPoint.CENTER;

    // Determines abilities of controller
    public enum DriveControlMode
    {
        JOYSTICKS,
        ROBOT_CENTRIC,
        LIMELIGHT,
        LIMELIGHT_AUTON,
        TRAJECTORY,
        HOLD,
        BALANCE, 
        ZERO, 
        HOLD_ANGLE, 
        HOLD_ANGLE_DRIVE,
        BRIDGE_VOLTAGE,
        VELOCITY;
    }

    public enum LimelightMode {
        NONE(0, true),
        RETROREFLECTIVE(0, true),
        APRIL_TAG(1, true),
        CUBE_INTAKE(0, false),
        AUTON_CUBE_TRACK(0, false);

        private int pipelineNum;
        private boolean isForward;

        LimelightMode(int pipelineNum, boolean isForward) {
            this.pipelineNum = pipelineNum;
            this.isForward = isForward;
        }

        public int getPipelineNum() {
            return pipelineNum;
        }

        public boolean isForwardPipeline() {
            return isForward;
        }
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
    public ProfiledPIDController rotationController = new ProfiledPIDController(2.0, 0.03, 0.02, constraints, 0.02);
    public ProfiledPIDController profiledLimelightController = new ProfiledPIDController(1.0, 0.03, 0.02, constraints, 0.02);
    public PidController limelightForwardAxisController = new PidController(new PidConstants(1.0, 0.002, 0.0));
    public PIDController ballTrackController = new PIDController(1.0, 0.03, 0.25, 0.02);
    private PidController balanceController = new PidController(new PidConstants(0.45, 0.0, 0.0));
    private PidController joystickRotateGyroController = new PidController(new PidConstants(.01, 0.002, 0.0));
    private PidController limelightStrafeController = new PidController(new PidConstants(.01, 0.002, 0.0));

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
            new PidConstants(15.0, 0.0, 0.0),
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

        if(!Robot.isCompetitionBot()){
            // Defaulted to Comp Bot so now we choose practice bot constants
            frontLeftOffset = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET;
            frontRightOffset = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET;
            backLeftOffset = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET;
            backRightOffset = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET;
        }

        modules = new SwerveModule[4];
        lastModuleAngle = new double[4];        
        tab = Shuffleboard.getTab("Drivetrain");

        createSwerveModulesAndShuffleboardInfo();
        odometryXEntry = tab.add("Odo X", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Odo Y", 0.0)
                .withPosition(1, 2)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Pose Angle", 0.0)
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();

        for(SwerveModule i : modules){
            i.setVoltageRamp(0.6);
        }

        limelightForwardAxisController.setOutputRange(-1, 1);
        limelightForwardAxisController.setShouldClearIntegralOnErrorSignChange(true);
        // limelightForwardAxisController.setTolerance(0.1);
        // limelightForwardAxisController.setIntegratorRange(-1.0, 1.0);

        // TODO change these pi values
        balanceController.setInputRange(0, Math.PI);
        balanceController.setContinuous(true);

        joystickRotateGyroController.setInputRange(-180, 180);
        joystickRotateGyroController.setOutputRange(-1, 1);
        joystickRotateGyroController.setShouldClearIntegralOnErrorSignChange(true);
        // joyStickRotateGyroController.setOutputRange(0, 1);
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
                        .withSize(1, 2),
                Mk4SwerveModuleHelper.GearRatio.WCP,
                Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                frontLeftOffset
        );
        SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withPosition(1, 0)
                        .withSize(1, 2),
                Mk4SwerveModuleHelper.GearRatio.WCP,
                Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                frontRightOffset
        );
        SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withPosition(2, 0)
                        .withSize(1, 2),
                Mk4SwerveModuleHelper.GearRatio.WCP,
                Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                backLeftOffset
        );
        SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withPosition(3, 0)
                        .withSize(1, 2),
                Mk4SwerveModuleHelper.GearRatio.WCP,
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
                .withPosition(3, 3)
                .withSize(1, 1);
        tab.addNumber("Trajectory Y", () -> {
                    if (follower.getLastState() == null) {
                        return 0.0;
                    }
                    return follower.getLastState().getPathState().getPosition().y;
                })
                .withPosition(4, 3)
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
            }).withPosition(5, 3)
              .withSize(1, 1);

              
        tab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity)
            .withPosition(6, 3)
            .withSize(1, 1);
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

    public void setLimelightMode(LimelightMode limelightMode){
        this.limelightMode = limelightMode;
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
        // For rotation axis, do not insert raw value
        return primaryController.getRightXAxis();
    }

    public void setLimelightOverride(boolean isOveridden){
        isLimelightOverride = isOveridden;
        if(isOveridden) {
            limelightForward.setLedMode(Limelight.LedMode.OFF);
        }
        else{
            limelightForward.setLedMode(Limelight.LedMode.ON);
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
        double rotationInput = getDriveRotationAxis().get(false);
        
        double rotationOutput = 0.0;
        if(Math.abs(getDriveForwardAxis().get()) < 0.2 && Math.abs(getDriveStrafeAxis().get()) < 0.2){
            // No correction when just driving/strafing
            
            // Drive input
            if(Math.abs(rotationInput) > Constants.DRIVE_ROTATION_JOYSTICK_DEADBAND) {
                // If not driving or strafing, but turning, allow the turn to occur.
                rotationOutput = getGyroRotationOutput(rotationInput);
            }
            // else leave rotationOutput at 0 so that drive wheels coast without turning
        }
        else {
            // Drive input
            rotationOutput = getGyroRotationOutput(rotationInput);
        }

        //Set the drive signal to a field-centric (last boolean parameter is true) joystick-based input.
        drive(new Vector2(
            getDriveForwardAxis().get(true)*(turbo ? 1.0 : Constants.TRANSLATIONAL_SCALAR), // Left Joystick YAxis
            getDriveStrafeAxis().get(true)*(turbo ? 1.0 : Constants.TRANSLATIONAL_SCALAR)),    // Left Joystick XAxis
            rotationOutput,
            true);  
        wasJustTurning = Math.abs(getDriveForwardAxis().get(true))<0.1 && Math.abs(getDriveStrafeAxis().get(true))<0.1 && Math.abs(getDriveRotationAxis().get(true))>0.1;                             
    }

    /**
     * If no input given and gyro autocorrect is on, output is given to rotate towards commandedPoseAngle
     * @param rotationInput Driver's rotation joystick, or 0 for hold-angle
     * @return [-1,1] rotational velocity for use in drive signal
     */
    public double getGyroRotationOutput(double rotationInput) {
        double rotationOutput = 0.0;
        if(Math.abs(rotationInput) <= Constants.DRIVE_ROTATION_JOYSTICK_DEADBAND)
        {
            // No command from rotation joystick
            if(RobotContainer.getInstance().getGyroAutoAdjustMode().getMode() == org.frcteam2910.c2020.util.GyroAutoChooser.Mode.On)
            {
                // Auto gyro correction if no turning
                double deltaAngleCurrToTarget = getLeastAngleDifference(getPose().rotation.toDegrees(), commandedPoseAngleDeg);
                SmartDashboard.putNumber("Delta Gyro To Cmd", deltaAngleCurrToTarget);
                // Radians version: joystickRotateGyroController.setSetpoint(Math.toRadians(-commandedPoseAngle) + getPose().rotation.toRadians());
                rotationOutput = joystickRotateGyroController.calculate(deltaAngleCurrToTarget, 0.02);
                    
                SmartDashboard.putNumber("Gyro Rotation Out", rotationOutput);

                if(Math.abs(rotationOutput) > 0.5) {
                    rotationOutput = Math.copySign(0.5, rotationOutput);
                }
                else if(Math.abs(rotationOutput) <= .15) {
                    rotationOutput = 0.0;
                }
            }
        }
        else {
            // Follow the rotation joystick's commands.
            commandedPoseAngleDeg = getPose().rotation.toDegrees();
            
            rotationOutput = getDriveRotationAxis().get(true) * Constants.ROTATIONAL_SCALAR;
        }
        return rotationOutput;
    }

    public void voltageDrive(){
        commandedPoseAngleDeg = 180;
        drive(new Vector2((voltageOutput/12), 0.0), getGyroRotationOutput(0), false);
    }

    public void rotationDrive(){
        // DriveControlMode is ROTATION/GYRO AUTO-CORRECT

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double deltaAngleCurrToTarget = getLeastAngleDifference(gyroscope.getAngle().toDegrees(), commandedPoseAngleDeg);
        
        double rotationOutput = getDriveRotationAxis().get(true)*Constants.ROTATIONAL_SCALAR;

        if(Math.abs(getDriveRotationAxis().get(true))<0.1){
            rotationOutput = joystickRotateGyroController.calculate(deltaAngleCurrToTarget, 0.02);
        }else{
            commandedPoseAngleDeg = gyroscope.getAngle().toDegrees();
        }
        
        if(Math.abs(getDriveForwardAxis().get(true))<0.1 && Math.abs(getDriveStrafeAxis().get(true))<0.1 && Math.abs(getDriveRotationAxis().get(true))<0.1){
            if(wasJustTurning && Math.abs(getDriveRotationAxis().get(true))<0.1){
                setDriveControlMode(DriveControlMode.ZERO);
                return;
            }
        }
        // Set the drive signal to a robot-centric joystick-based input for drive & strafe only (not rotation/chassis angle).
        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
    
        wasJustTurning = Math.abs(getDriveForwardAxis().get(true))<0.1 && Math.abs(getDriveStrafeAxis().get(true))<0.1 && Math.abs(getDriveRotationAxis().get(true))>0.1;                             
    }

    public void robotCentricDrive(){
        // DriveControlMode is ROBOT_CENTRIC

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        // Set the drive signal to a robot-centric (last boolean parameter is false) joystick-based input.
        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), getDriveRotationAxis().get(true), false);
    }

    public void limelightDrive(){
        // DriveControlMode is LIMELIGHT

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double driveForwardOutput = getDriveForwardAxis().get(true)*(turbo ? 1.0 : Constants.TRANSLATIONAL_SCALAR);
        double strafeOutput = getDriveStrafeAxis().get(true)*(turbo ? 1.0 : Constants.TRANSLATIONAL_SCALAR);
        double rotationOutputCommanded = getGyroRotationOutput(getDriveRotationAxis().get());

        if(limelightMode == LimelightMode.NONE) {
            // Set the drive signal to a field-centric joystick-based input when we don't see a ball.
            // Also allow rotation from the right joystick on the driver controller.
            drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, true);
            primaryController.setRumble(RumbleType.kLeftRumble, 0.0);
            primaryController.setRumble(RumbleType.kRightRumble, 0.0);
            return;
        }

        boolean forward = limelightMode.isForwardPipeline();
        if(forward) {
            limelightForward.setPipeline(limelightMode.getPipelineNum());
        }
        else {
            limelightBack.setPipeline(limelightMode.getPipelineNum());
        }

        // Set the drive signal to a field-centric joystick-based input when we see a target.
        if(forward) {
            if(limelightForward.hasTarget()) {
                limelightStrafeController.setSetpoint(-12);

    
                // Last boolean in drive() is true for field-oriented or false for robot-centric left joystick
                commandedPoseAngleDeg = 0;
                // We must modify the above parameter to lock to 0 so strafing while rotated is not allowed
                rotationOutputCommanded = getGyroRotationOutput(0);
                strafeOutput = limelightStrafeController.calculate(limelightForward.getTargetHorizOffset(), 0.02);
                drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, true);
                // primaryController.setRumble(RumbleType.kLeftRumble, 0.25);
                // primaryController.setRumble(RumbleType.kRightRumble, 0.25);
            }
            else {
                // Set the drive signal to a field-centric joystick-based input when we don't see a target.
                // Also allow rotation from the right joystick on the driver controller.
                drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, true);
                primaryController.setRumble(RumbleType.kLeftRumble, 0.0);
                primaryController.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }
        else {
            if(limelightBack.hasTarget()) {
                // This page intentionally left blank
                
            }
            else {
                // Set the drive signal to a field-centric joystick-based input when we don't see a target.
                // Also allow rotation from the right joystick on the driver controller.
                drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, true);
                primaryController.setRumble(RumbleType.kLeftRumble, 0.0);
                primaryController.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }
    }

    public void autonLimelightDrive(){
        // DriveControlMode is LIMELIGHT

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double driveForwardOutput = 0.0;
        double strafeOutput = 0.0;
        commandedPoseAngleDeg = getPose().rotation.toDegrees();
        double rotationOutputCommanded = getGyroRotationOutput(0.0);

        if(limelightMode == LimelightMode.NONE) {
            // Set the drive signal to a field-centric joystick-based input when we don't see a ball.
            // Also allow rotation from the right joystick on the driver controller.
            drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, true);
            return;
        }

        boolean forward = limelightMode.isForwardPipeline();
        if(forward) {
            limelightForward.setPipeline(limelightMode.getPipelineNum());
        }
        else {
            limelightBack.setPipeline(limelightMode.getPipelineNum());
        }

        // Set the drive signal to a field-centric joystick-based input when we see a target.
        if(forward) {
            // if(limelightForward.hasTarget()) {
            //     limelightStrafeController.setSetpoint(0.0);
    
            //     // Last boolean in drive() is true for field-oriented or false for robot-centric left joystick
            //     commandedPoseAngleDeg = 0;
            //     // We must modify the above parameter to lock to 0 so strafing while rotated is not allowed
            //     rotationOutputCommanded = getGyroRotationOutput(0);
            //     strafeOutput = limelightStrafeController.calculate(limelightForward.getTargetHorizOffset(), 0.02);
            //     drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, true);
            // }
            // else {
            //     // Set the drive signal to a field-centric joystick-based input when we don't see a target.
            //     // Also allow rotation from the right joystick on the driver controller.
            //     drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, true);
            // }
        }
        else {
            if(limelightBack.hasTarget()) {
                if(limelightMode == LimelightMode.CUBE_INTAKE) {
                    // limelightForwardAxisController.setSetpoint(-15);
                    // driveForwardOutput = limelightForwardAxisController.calculate(limelightBack.getTargetVertOffset(), 0.02);
                    driveForwardOutput = -0.2;
                    strafeOutput = 0.0;
                    joystickRotateGyroController.setSetpoint(0);
                    rotationOutputCommanded = joystickRotateGyroController.calculate(limelightBack.getTargetHorizOffset(), 0.02); 
                    drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, false);
                }
            }
            else {
                // Set the drive signal to a field-centric joystick-based input when we don't see a target.
                // Also allow rotation from the right joystick on the driver controller.
                drive(new Vector2(driveForwardOutput, strafeOutput), rotationOutputCommanded, true);
            }
        }
    }

    public double getStartDegrees(){
        return this.startDegrees;
    }

    public void setSlowBalance(boolean set){
        this.slowBalance = set;
    }

    public void balanceOutDrive() {
        if(!slowBalance){
            boolean tiltedBackward = (getRoll() > 180);
            double degreesAwayFromBalance = tiltedBackward ? (360 - getRoll()) : getRoll();
            drive(new Vector2((tiltedBackward?1:-1)*0.25, 0.0), 0.0, false);
        }
        else{
            balanceController.reset();
            balanceController.setSetpoint(0);
            boolean tiltedBackward = (getRoll() > 180);
            double pitchOutput = tiltedBackward ? -1 : 1;
            double degreesAwayFromBalance = tiltedBackward ? (360 - getRoll()) : getRoll();
            if(degreesAwayFromBalance < Constants.BALANCE_DEADBAND){
                degreesAwayFromBalance = 0;
                if(!balanceTimer.hasElapsed(0.1)){
                    balanceTimer.start();
                    balanceController.integralAccum=0;
                }    
            }
            else{
                balanceTimer.stop();
                balanceTimer.reset();
            } 

            double forwardAxisOutput = balanceController.calculate(Math.toRadians(degreesAwayFromBalance), 0.02);
            if(degreesAwayFromBalance > Constants.BALANCE_DEADBAND)
                forwardAxisOutput/=1.3;  
            drive(new Vector2(pitchOutput * forwardAxisOutput, 0.0), 0.0, false);
        }
        // DriveControlMode is BALANCE
        // balanceController.reset();
        // balanceController.setSetpoint(0);
        // //TODO change to field oriented
        // double angle = getPose().rotation.toDegrees();
        // boolean roll = (angle>60 && angle<120)||(angle>240 && angle<300);
        // boolean pitch = roll?false:(angle>330 || angle<30)||(angle>150 && angle<210);
        // boolean tiltedBackward = false;
        // double invertOutput = 1.0;
        // double degreesAwayFromBalance = 0;

        // SmartDashboard.putBoolean("isRoll", roll);
        // SmartDashboard.putBoolean("isPitch", pitch);
        // //if(degreesAwayFromBalance>1)
        // //    degreesAwayFromBalance*=degreesAwayFromBalance;
        // if(roll){
        //     degreesAwayFromBalance = getRollDegreesOffLevel();
        // }else if(pitch){
        //     degreesAwayFromBalance = getPitchDegreesOffLevel();
        // }else{
        //     degreesAwayFromBalance = 0.0; 
        //     setDriveControlMode(DriveControlMode.JOYSTICKS);
        //     return;       
        // }

        // SmartDashboard.putNumber("degrees away", degreesAwayFromBalance);

        // // double x = getPose().translation.x;

        // // if(side==SideMode.RED){
        // //     if(x<-210){
        // //         invertOutput = -1.0;
        // //     }
        // //     else if(x>-210){
        // //         invertOutput = 1.0;
        // //     }
        // // }
        // // else if(side==SideMode.BLUE){    
        // //     if(x>210){
        // //         invertOutput = -1.0;
        // //     }
        // //     else if(x<210){
        // //         invertOutput = 1.0;
        // //     }
        // // }

        // double forwardAxisOutput = balanceController.calculate(Math.toRadians(degreesAwayFromBalance), 0.02);
        
        // if(degreesAwayFromBalance < Constants.BALANCE_DEADBAND)
        // {
        //     forwardAxisOutput *= 1.2;
        //     if(!balanceTimer.hasElapsed(0.1))
        //         balanceTimer.start();
        // }
        // else if(degreesAwayFromBalance>10)
        // {
        //     balanceTimer.stop();
        //     balanceTimer.reset();
        //     forwardAxisOutput *= 1.5;
        // }
        // else
        // {
        //     balanceTimer.stop();
        //     balanceTimer.reset();
        //     forwardAxisOutput *= .8;
        // }

        // drive(new Vector2(invertOutput * forwardAxisOutput, 0.0), 0.0, false);
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
            // Can't allow discrepancy between gyro and commandedPoseAngle
            commandedPoseAngleDeg=angle.toDegrees();
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

    public void setSteerBrake(){
        for(SwerveModule i : modules){
            i.setSteerNeutralMode(NeutralMode.Brake);
        }
    }

    public void setBridgeDriveVoltage(double voltage){
        this.voltageOutput = voltage;
    }

    public void setBridgeDriveAngle(double angle){
        this.voltageSteerAngle = angle;
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

    public boolean atRotationTarget(){
        
        if(rotationController.atGoal()){
            //System.out.println("Reached target");
        }
        return rotationController.atGoal();
    }
    
    public void setBalanceStartDegrees(double startDegrees){
        this.startDegrees = startDegrees;
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

        // if(Math.abs(getDriveForwardAxis().get(true))>0.1 || Math.abs(getDriveStrafeAxis().get(true))>0.1 || Math.abs(getDriveRotationAxis().get(true))>0.1){
        //     setDriveControlMode(DriveControlMode.JOYSTICKS);
        //     wasJustTurning = false;
        // }   

        DriveControlMode i_controlMode = getDriveControlMode();

        HolonomicDriveSignal currentDriveSignal = null;

        //System.out.println("Control Mode: " + driveControlMode);

        if(RobotContainer.getInstance().getGyroAutoAdjustMode().getMode() == org.frcteam2910.c2020.util.GyroAutoChooser.Mode.On){
            if(i_controlMode==DriveControlMode.JOYSTICKS){
                i_controlMode = DriveControlMode.HOLD_ANGLE_DRIVE;
                setDriveControlMode(i_controlMode);
            }
        }

        switch(i_controlMode){
            case JOYSTICKS:
                joystickDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case HOLD_ANGLE_DRIVE:
                joystickDrive();
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
            case LIMELIGHT:
                limelightDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case LIMELIGHT_AUTON:
                autonLimelightDrive();
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
            case HOLD_ANGLE :
                updateModulesHoldAngle();    
                break;
            case HOLD:
                updateModulesHold();
                break;
            case ZERO:
                setModulesAngle(0.0);
                break;
            case VELOCITY:
                double metersPerSec = 1.0;
                setModuleVelocity(metersPerSec);
            case BRIDGE_VOLTAGE:
                voltageDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;    
        }

        if(driveControlMode != DriveControlMode.HOLD && driveControlMode != DriveControlMode.VELOCITY) {
            // Tell all swerve modules their new targets based on the kinematics.
            updateModules(currentDriveSignal);
        }
    }

    public void setTurbo(boolean turbo){
        this.turbo = turbo;
    }

    public double getPitch(){
        synchronized(sensorLock) {
            return gyroscope.getPitch(); //- 2; // -2 for bias when level
        }
    }

    public double getRoll(){
        synchronized(sensorLock) {
            return gyroscope.getRoll(); //+ 0.3; // +0.3 for bias when level
        }
    }
    public double getYawDegreesTargetOffset() {
        double yaw = getPose().rotation.toDegrees();
        return getLeastAngleDifference(yaw, 0);
    }

    public double getPitchDegreesOffLevel(){
        double pitch = getPitch();
        return Math.abs(getLeastAngleDifference(pitch, 0));
    }

    public double getRollDegreesOffLevel(){
        double roll = getRoll();
        return Math.abs(getLeastAngleDifference(roll, 0));
    }

    /**
     * @param currPoseAngle
     * @param targetAngle
     * @return An angle within [-180, 180) degrees dictating which way to turn to achieve target angle.
     */
    public static double getLeastAngleDifference(double currPoseAngle, double targetAngle){
        currPoseAngle %= 360;
        targetAngle %= 360;
        if(targetAngle < currPoseAngle) {
            targetAngle += 360;
        }
        double diff = currPoseAngle - targetAngle;
        if(diff < 0 && Math.abs(diff) >= 180) {
            // Negative diff needs to wrap to become a positive diff (currPose < targetAngle)
            diff = diff + 360;
        }
        else if (diff > 180) {
            // Positive diff (currPose > targetAngle)
            diff = 360 - diff;
        }
        return diff;
    }

    private void updateOdometry(double time, double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getSteerAngle())).scale(module.getDriveVelocity() * Constants.INCHES_PER_METER * WHEEL_DIAMETER_INCHES / 4.0);
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
            if(true/*getPitchDegreesOffLevel()<5 && getRollDegreesOffLevel()<5*/){
                // On the ground, update odometry
                this.pose = swerveOdometry.update(swerveKinematics, angle, dt, moduleVelocities);
            }
            // else {
            //     // Not on the ground
            //     double xacc = gyroscope.getAccels()[0]/(double)Short.MAX_VALUE;
            //     double yacc = gyroscope.getAccels()[1]/(double)Short.MAX_VALUE;
            //     if((xacc < .12 && getPitchDegreesOffLevel()>Constants.BALANCE_DEADBAND+2)
            //         || (yacc < .12 && getRollDegreesOffLevel()>Constants.BALANCE_DEADBAND+2)) {
            //         Vector2[] zeros = {new Vector2(0,0),new Vector2(0,0),new Vector2(0,0),new Vector2(0,0)};
            //         this.pose = swerveOdometry.update(swerveKinematics, angle, dt, zeros);
            //     }
            //     else this.pose = swerveOdometry.update(swerveKinematics, angle, dt, moduleVelocities);
            // }    
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

    private void updateModulesHoldAngle() {
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.set(0, lastModuleAngle[i]);
        }
    }


    public void setModulesAngle(double angle) {
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.set(0, Math.toRadians(angle));
        }
    }

    public void setModuleVelocity(double metersPerSec) {
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.setVelocity(metersPerSec, lastModuleAngle[i]);
        }
    }

    public void zeroGyro(){
        synchronized(sensorLock) {
            gyroscope.zeroGyro();
        }
    }
    //#endregion

    @Override
    public void periodic() {
        // Must update the Tx/Ty filter to provide it samples for calculation
        limelightForward.updateTxFilter();
        limelightForward.updateTyFilter();

        limelightBack.updateTxFilter();
        limelightBack.updateTyFilter();

        // Update SmartDashboard/Shuffleboard data
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(pose.rotation.toDegrees());

        if(DriverStation.getMatchTime()<5){
            setDriveBrake();
            setSteerBrake();
        }
        else{
            setDriveCoast();
        }


        SmartDashboard.putString("Drive Mode", getDriveControlMode().name());
        SmartDashboard.putNumber("Yaw Target", commandedPoseAngleDeg);
        SmartDashboard.putNumber("Yaw Curr", getPose().rotation.toDegrees());
        // SmartDashboard.putNumber("X", pose.translation.x);
        // SmartDashboard.putNumber("Y", pose.translation.y);
        // SmartDashboard.putBoolean("was just turning", wasJustTurning);
        SmartDashboard.putNumber("Pitch", getPitchDegreesOffLevel());
        // SmartDashboard.putString("Translation Drive", driveSignal.getTranslation().x+"\n"+driveSignal.getTranslation().y);
        // SmartDashboard.putNumber("Rotation Drive", driveSignal.getRotation());
        SmartDashboard.putNumber("Roll", getRollDegreesOffLevel());
        SmartDashboard.putNumber("raw pitch", getPitch());
        SmartDashboard.putNumber("raw roll", getRoll());
        // SmartDashboard.putNumber("blanace timer length", balanceTimer.get());
        // SmartDashboard.putBoolean("turbo", turbo);
        // SmartDashboard.putNumber("drive voltage", voltageOutput);

        synchronized(sensorLock) {
            SmartDashboard.putNumber("gravity vector", gyroscope.getGravityVector()[0]);
            SmartDashboard.putNumber("gravity vector1", gyroscope.getGravityVector()[1]);
            SmartDashboard.putNumber("gravity vector2", gyroscope.getGravityVector()[2]);
        }
    }
}
