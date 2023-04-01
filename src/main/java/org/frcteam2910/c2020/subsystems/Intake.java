package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Robot;
import org.frcteam2910.c2020.Servo;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    public enum IntakeControlMode{
        HOLD, MANUAL
    }

    //servos
    private final Servo rightServo = new Servo(Constants.RIGHT_SERVO_PORT);
    private final Servo leftServo = new Servo(Constants.LEFT_SERVO_PORT);

    //falcons
    private TalonFX intakeMotor;
    private TalonFX cubeIntakeRollerMotor;
    private TalonFX cubeIntakeLiftMotor;

    //sensors
    private DigitalInput cubeSensor;
    private DigitalInput coneSensor;
    private DigitalInput cubeRollerSensor;

    //conversions
    
    //misc
    private int kIntakeVelocitySlot = 0;
    private int kIntakePositionSlot = 1;
    private boolean firstSet = false;
    private double _deployPosDegreesOffset = 0.0;
    public boolean stopRollingOnTriggeredCubeIntakeDIO = false;
    public boolean stopRollingOnTriggeredArmIntakeDIO = false;
    private long lastSysMillisTriggeredDIO = Long.MIN_VALUE;

    boolean hasSetIntakeZero = false;
    private IntakeControlMode controlMode = IntakeControlMode.MANUAL;

    private static Intake INSTANCE=null;

    //#region Constructors
    public static Intake getInstance(){
        if(INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }
    
    private Intake(){
        stopRollingOnTriggeredArmIntakeDIO = false;
        stopRollingOnTriggeredCubeIntakeDIO = false;

        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_PORT, "rio");
        cubeIntakeRollerMotor = new TalonFX(Constants.CUBE_INTAKE_ROLLER_MOTOR_PORT, "rio");
        cubeIntakeLiftMotor = new TalonFX(Constants.CUBE_INTAKE_DEPLOY_MOTOR_PORT, "Drivetrain");
        intakeMotor.setInverted(false);
        cubeIntakeRollerMotor.setInverted(false);
        cubeIntakeLiftMotor.setInverted(false);
        leftServo.setInverted(false);
        rightServo.setInverted(true);

        cubeIntakeLiftMotor.setNeutralMode(NeutralMode.Brake);
        
        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 80.0;
        statorCurrentConfigs.enable = true;
        intakeMotor.configStatorCurrentLimit(statorCurrentConfigs);  
        cubeIntakeRollerMotor.configStatorCurrentLimit(statorCurrentConfigs);
        statorCurrentConfigs.currentLimit = 40.0;
        cubeIntakeLiftMotor.configStatorCurrentLimit(statorCurrentConfigs);      

        intakeMotor.config_kF(kIntakeVelocitySlot, 0.0);
        intakeMotor.config_kP(kIntakeVelocitySlot, 0.10);
        intakeMotor.config_kI(kIntakeVelocitySlot, 0.0001);
        intakeMotor.config_kD(kIntakeVelocitySlot, 0.0);
        intakeMotor.config_IntegralZone(kIntakeVelocitySlot, (int)this.ArmRoller_RpmToVelocityTicks(200));

        intakeMotor.config_kF(kIntakePositionSlot, 0.0);
        intakeMotor.config_kP(kIntakePositionSlot, 0.1);
        intakeMotor.config_kI(kIntakePositionSlot, 0.0);
        intakeMotor.config_kD(kIntakePositionSlot, 0.0);

        cubeIntakeRollerMotor.config_kF(kIntakeVelocitySlot, 0.0);
        cubeIntakeRollerMotor.config_kP(kIntakeVelocitySlot, -0.10);
        cubeIntakeRollerMotor.config_kI(kIntakeVelocitySlot, 0.0004);
        cubeIntakeRollerMotor.config_kD(kIntakeVelocitySlot, 0.0);
        //cubeIntakeMotor.config_IntegralZone(kIntakeVelocitySlot, (int)this.CubeRoller_RpmToVelocityTicks(200));

        cubeIntakeRollerMotor.config_kF(kIntakePositionSlot, 0.0);
        cubeIntakeRollerMotor.config_kP(kIntakePositionSlot, -0.07);
        cubeIntakeRollerMotor.config_kI(kIntakePositionSlot, -0.00007);
        cubeIntakeRollerMotor.config_kD(kIntakePositionSlot, 0.0);

        cubeIntakeLiftMotor.config_kF(kIntakePositionSlot, 0.00);
        cubeIntakeLiftMotor.config_kP(kIntakePositionSlot, 0.06);
        cubeIntakeLiftMotor.config_kI(kIntakePositionSlot, 0.00007);
        cubeIntakeLiftMotor.config_kD(kIntakePositionSlot, 0.0);

        cubeSensor = new DigitalInput(1);
        coneSensor = new DigitalInput(0);
        cubeRollerSensor = new DigitalInput(2);
    }
    //#endregion
    
    //#region Class Methods
        //#region servo
    public void setServoPosition(double position){
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
    //#endregion
        //#region intake
        public void setControlMode(IntakeControlMode c){
            this.controlMode = c;
        }

        public void setIntakeSetZero(boolean zero){
            this.hasSetIntakeZero = zero;
        }

        public void setArmIntakeHold(){
                controlMode = IntakeControlMode.HOLD;
                intakeMotor.selectProfileSlot(kIntakePositionSlot, 0);
                intakeMotor.set(TalonFXControlMode.Position, intakeMotor.getSelectedSensorPosition());
                SmartDashboard.putNumber("set position", intakeMotor.getSelectedSensorPosition());
        }

        public void setArmIntakeSpeed(double speed) {
            firstSet = false;
            controlMode = IntakeControlMode.MANUAL;
            this.intakeMotor.set(ControlMode.PercentOutput, speed);
        }
        
        /** @see SetIntakeRPM */
        public void setArmIntakeRPM(double rpm) {
            firstSet = false;
            controlMode = IntakeControlMode.MANUAL;
            intakeMotor.selectProfileSlot(kIntakeVelocitySlot, 0);
            intakeMotor.set(ControlMode.Velocity, this.ArmRoller_RpmToVelocityTicks(rpm));
        }

        /** @see SetIntakeDeployPosition */
        public void setCubeRollerRPM(double rpm) {
            if(Math.floor(rpm) != 0){
                cubeIntakeRollerMotor.selectProfileSlot(kIntakeVelocitySlot, 0);
                cubeIntakeRollerMotor.set(TalonFXControlMode.Velocity, this.CubeRoller_RpmToVelocityTicks(rpm));
            }
            else{
                cubeIntakeRollerMotor.selectProfileSlot(kIntakePositionSlot, 0);
                cubeIntakeRollerMotor.set(TalonFXControlMode.Position, this.cubeIntakeRollerMotor.getSelectedSensorPosition());
            }
        }

        public void clearDeployIntegrator() {
            cubeIntakeLiftMotor.setIntegralAccumulator(0);
        }

        public void setCubeIntakeDeployTargetPosition(double positionDegrees) {
            // if(positionDegrees<getCubeIntakeDeployDegrees()){
            //     cubeIntakeLiftMotor.config_kF(kIntakePositionSlot, 0.0);
            //     cubeIntakeLiftMotor.config_kP(kIntakePositionSlot, 0.02);
            // }else{
            //     cubeIntakeLiftMotor.config_kF(kIntakePositionSlot, -0.006);
            //     cubeIntakeLiftMotor.config_kP(kIntakePositionSlot, 0.01);
            // // }
            cubeIntakeLiftMotor.setIntegralAccumulator(0);
            cubeIntakeLiftMotor.selectProfileSlot(kIntakePositionSlot, 0);
            cubeIntakeLiftMotor.set(TalonFXControlMode.Position, getCubeIntakeDeployDegreesTicks(positionDegrees));
            // if(positionDegrees==110){
            //     cubeIntakeLiftMotor.set(ControlMode.PercentOutput, 0.2);
            // }
            // else{
            //     cubeIntakeLiftMotor.set(ControlMode.PercentOutput, -0.2);
            // }
        }

        public boolean withinCubeDeployTarget(double tolerance, double position) {
            return Math.abs(getCubeIntakeDeployDegrees() - position) < tolerance;
        }

        public void setCubeIntakeDeployHome(double deployPosDegreesOffset) {
            _deployPosDegreesOffset = deployPosDegreesOffset;
            cubeIntakeLiftMotor.selectProfileSlot(kIntakePositionSlot, 0);
            cubeIntakeLiftMotor.setSelectedSensorPosition(0);
        }

        // These are for velocity units
        public double ArmRoller_RpmToVelocityTicks(double rpm) {
            return (rpm * Constants.ARM_INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS) / (10*60);
        }
        public double ArmRoller_VelocityTicksToRpm(double ticksPer100ms) {
            return (ticksPer100ms / Constants.ARM_INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS) * (10*60);
        }

        // These are for velocity units
        public double CubeRoller_RpmToVelocityTicks(double rpm) {
            return (rpm * Constants.CUBE_INTAKE_ROLLER_MOTOR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION) / (10*60);
        }
        public double CubeRoller_VelocityTicksToRpm(double ticksPer100ms) {
            return (ticksPer100ms / (Constants.CUBE_INTAKE_ROLLER_MOTOR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION)) * (10*60);
        }

        public void resetIntakeDIOTimestamp() {
            lastSysMillisTriggeredDIO = Long.MIN_VALUE;
        }

        public double getCubeRollerRPM() {
            return this.CubeRoller_VelocityTicksToRpm(cubeIntakeRollerMotor.getSelectedSensorVelocity());
        }

        public double getCubeIntakeDeployDegrees() {
            return (cubeIntakeLiftMotor.getSelectedSensorPosition() / Constants.CUBE_INTAKE_DEPLOY_ONE_DEGREE_IN_ENCODER_TICKS) + _deployPosDegreesOffset;
        }

        public double getCubeIntakeDeployDegreesTicks(double degrees) {
            return (degrees * Constants.CUBE_INTAKE_DEPLOY_ONE_DEGREE_IN_ENCODER_TICKS);
        }

        public DigitalInput getCubeSensor() {
            return cubeSensor;
        }

        public DigitalInput getConeSensor() {
            return coneSensor;
        }
        //#endregion
    //#endregion

    boolean setRPMZero = false;

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("DIO Cone", coneSensor.get());
        SmartDashboard.putBoolean("DIO Cube", cubeSensor.get());
        SmartDashboard.putBoolean("DIO Cube Roll", cubeRollerSensor.get());
        // SmartDashboard.putBoolean("set intake zero", hasSetIntakeZero);
        SmartDashboard.putNumber("intake motor current", intakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("intake motor position", intakeMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("intake deploy position", getCubeIntakeDeployDegrees());
        SmartDashboard.putNumber("cube intake roller rpm", getCubeRollerRPM());
        SmartDashboard.putNumber("intake lift voltage", cubeIntakeLiftMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("cube intake current", cubeIntakeLiftMotor.getStatorCurrent());


        // if(cubeSensor.get()){
        //     setArmIntakeHold();
        // }
        if(stopRollingOnTriggeredCubeIntakeDIO){
            double delayFactorSec = 0.25;
            if(cubeRollerSensor.get() && !setRPMZero){
                // if(lastSysMillisTriggeredDIO <= 0){
                //     lastSysMillisTriggeredDIO = System.currentTimeMillis();
                // }
                setCubeRollerRPM(0);
                setRPMZero = true;
            }
            else {
                setRPMZero = false;
            }
            // if(System.currentTimeMillis() - lastSysMillisTriggeredDIO > (int)(delayFactorSec * 1000)) {
            //     // We're beyond the delay factor, time to act
            //     setCubeRollerRPM(0);
            // }
        }

        if(stopRollingOnTriggeredArmIntakeDIO) {
            // While holding right trigger (cube intake), if detect something where the cone would go,
            // override any commands to move intake rollers.
                double delayFactorSec = 0.25;
            if(cubeSensor.get() && !setRPMZero){
                // if(lastSysMillisTriggeredDIO <= 0){
                //     lastSysMillisTriggeredDIO = System.currentTimeMillis();
                // }
                setCubeRollerRPM(0);
                setArmIntakeHold();
                setRPMZero = true;
            }
            else {
                setRPMZero = false;
            }
            // if(System.currentTimeMillis() - lastSysMillisTriggeredDIO > (int)(delayFactorSec * 1000)) {
            //     // We're beyond the delay factor, time to act
            //     setCubeRollerRPM(0);
            //     setArmIntakeHold();
            // }
        }
    }

	public IntakeControlMode getControlMode() {
		return controlMode;
	}
}