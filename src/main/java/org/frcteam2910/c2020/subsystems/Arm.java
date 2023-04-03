package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.util.ScoreMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Arm implements Subsystem{
    public enum ArmControlMode{
        MANUAL, HOLD
    }

    //motors
    private TalonFX armRotationMotor;
    private TalonFX armTranslationMotor;

    //sensors
    //private CANCoder armExternalCANCoder = new CANCoder(Constants.ARM_EXTERNAL_CANCODER_PORT);

    //conversions
    private double DRUM_DIAMETER = 0.54134;
    private double TRANSLATIONAL_ROTATIONS_TO_INCHES = Math.PI * DRUM_DIAMETER;
    private double ARM_INCHES_TO_ENCODER_TICKS = Constants.ARM_EXTENDER_GEAR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION/TRANSLATIONAL_ROTATIONS_TO_INCHES;//Constants.ARM_TRANSLATIONAL_GEAR_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * TRANSLATIONAL_ROTATIONS_TO_INCHES;

    //misc
    private double inchesOffset;
    private double targetInches = 0.0;
    private double targetInchesTicks = 0;
    private double manualTranslationSpeed;
    private double degreesOffset;
    private double targetDegreesTicks;
    private double manualRotationSpeed;
    public ScoreMode scoreMode = ScoreMode.ZERO;

    private ArmControlMode extenderControlMode = ArmControlMode.HOLD;
    private ArmControlMode rotationControlMode = ArmControlMode.HOLD;

    private static Arm INSTANCE;

    //#region Constructors
    public static Arm getInstance(){
        if(INSTANCE == null) {
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }
    
    private Arm(){
        armRotationMotor = new TalonFX(Constants.ARM_ROTATION_MOTOR_PORT, "Drivetrain");
        armTranslationMotor = new TalonFX(Constants.ARM_TRANSLATIONAL_MOTOR_PORT);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        //armRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        armRotationMotor.configAllSettings(configs);
        armTranslationMotor.configAllSettings(configs);
        armTranslationMotor.setNeutralMode(NeutralMode.Brake);
        armTranslationMotor.setInverted(false);

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorDirection =  false;
        //armExternalCANCoder.configAllSettings(config);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 40.0;
        statorCurrentConfigs.enable = true;
        armRotationMotor.configStatorCurrentLimit(statorCurrentConfigs);

        armTranslationMotor.configStatorCurrentLimit(statorCurrentConfigs);

        armRotationMotor.setNeutralMode(NeutralMode.Brake);
        armTranslationMotor.setNeutralMode(NeutralMode.Brake);

        armRotationMotor.configMotionCruiseVelocity(20000);
        armRotationMotor.configMotionAcceleration(28000);
        armRotationMotor.configMotionSCurveStrength(4);

        armTranslationMotor.configMotionCruiseVelocity(20000);
        armTranslationMotor.configMotionAcceleration(28000);
        armTranslationMotor.configMotionSCurveStrength(4);

        armRotationMotor.config_kF(0, 0.0);
        armRotationMotor.config_kP(0, 0.03);
        armRotationMotor.config_kI(0, 0.00001);
        armRotationMotor.config_kD(0, 0.0);

        armTranslationMotor.config_kF(0, 0.0);
        armTranslationMotor.config_kP(0, 0.03); //0.06
        armTranslationMotor.config_kI(0, 0.0);//0.0000001
        armTranslationMotor.config_kD(0, 0.0);
    }
    //#endregion
    
    //#region rotation Methods
    public boolean withinAngle(double tolerance, double angle){
        // SmartDashboard.putNumber("angle checking", angle);
        // SmartDashboard.putNumber("result", Math.abs(getArmDegreesIntegrated()-angle));
        return Math.abs(getArmDegreesIntegrated()-angle) < tolerance;
    }

    public boolean withinCubeTarget(){
        return Math.abs(getArmInches()-targetInches)<0.75;
    }

    public void setRotationControlMode(ArmControlMode mode){
        rotationControlMode = mode;
    }

    public ScoreMode getScoreMode(){
        return this.scoreMode;
    }

    public void setScoreMode(ScoreMode sm){
        this.scoreMode = sm;
    }

    public double getArmDegreesIntegrated(){
        return (armRotationMotor.getSelectedSensorPosition() / Constants.ARM_ROTATOR_ONE_DEGREE_TO_INTEGRATED_ENCODER_TICKS) + degreesOffset;
    }

    // public double getArmDegreesExternal(){
    //     return (armExternalCANCoder.getPosition() / Constants.ARM_ROTATOR_ONE_DEGREE_TO_EXTERNAL_ENCODER_TICKS) + degreesOffset;
    // }

    public double getArmDegreesEncoderTicks(double degrees){
        return degrees * Constants.ARM_ROTATOR_ONE_DEGREE_TO_INTEGRATED_ENCODER_TICKS;
    }

    public void setArmDegreesZero(double offset){
        degreesOffset = offset;
        targetDegreesTicks = 0;
        armRotationMotor.setSelectedSensorPosition(0);
        // armExternalCANCoder.setPosition(0);
    }

    public double limitArmDegrees(double targetDegrees){
        if(targetDegrees < Constants.ARM_ROTATOR_MIN_ROTATION_DEGREES){
            return Constants.ARM_ROTATOR_MIN_ROTATION_DEGREES;
        }else if(targetDegrees > Constants.ARM_ROTATOR_MAX_ROTATION_DEGREES){
            return Constants.ARM_ROTATOR_MAX_ROTATION_DEGREES;
        }

        return targetDegrees;   
    }

    public synchronized void setArmDegreesPositionAbsolute(double degrees) {
        rotationControlMode = ArmControlMode.HOLD;
        armRotationMotor.selectProfileSlot(0, 0);
        armRotationMotor.setIntegralAccumulator(0);
        targetDegreesTicks = getArmDegreesEncoderTicks(limitArmDegrees(degrees));
        // armRotationMotor.set(ControlMode.Position, targetDegreesTicks, DemandType.ArbitraryFeedForward, 0.03);
        armRotationMotor.set(ControlMode.MotionMagic, targetDegreesTicks);
    }

    public synchronized void setRotationSpeed(double speed) {
        manualRotationSpeed = speed;

        rotationControlMode = ArmControlMode.MANUAL;
        if (getArmDegreesIntegrated() < Constants.ARM_ROTATOR_MIN_ROTATION_DEGREES && speed < 0.0) {
            speed = 0;
        } else if (getArmDegreesIntegrated() > Constants.ARM_ROTATOR_MAX_ROTATION_DEGREES && speed > 0.0) {
            speed = 0;
        }

        if(Math.abs(speed) > 0.5) {
            speed = Math.copySign(0.5, speed);
        }

        armRotationMotor.set(ControlMode.PercentOutput, speed);
    }

    public synchronized void setRotationHold(){
        // if(firstHoldSet){
        //     firstHoldSet = false;
        //     targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(getArmDegrees()));
        // }
        rotationControlMode = ArmControlMode.HOLD;
        armRotationMotor.selectProfileSlot(0, 0);    
        //targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(getArmDegrees()));
        armRotationMotor.set(ControlMode.Position, targetDegreesTicks);
    }

    public double getTargetDegrees(){
        return targetDegreesTicks/Constants.ARM_ROTATOR_ONE_DEGREE_TO_INTEGRATED_ENCODER_TICKS;
    }   

    public void setMotorNeutralMode(NeutralMode nm) {
        armRotationMotor.setNeutralMode(nm);
        armTranslationMotor.setNeutralMode(nm);
    }
    //#endregion

    //#region arm
    public boolean withinInches(double tolerance, double inches){
        return Math.abs(getArmInches()-inches) < tolerance;
    }    

    public void setExtenderControlMode(ArmControlMode mode){
        extenderControlMode = mode;
    }

    public double getTranslationalRotations(){
        return armTranslationMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / Constants.ARM_EXTENDER_GEAR_RATIO;
    }

    public double getArmInches(){
        return (armTranslationMotor.getSelectedSensorPosition() / ARM_INCHES_TO_ENCODER_TICKS);
    }

    public double getArmInchesEncoderTicksAbsolute(double inches){
        return (inches * ARM_INCHES_TO_ENCODER_TICKS);
    }

    public void setArmInchesZero(double inchesOffset){
        this.inchesOffset = inchesOffset;
        armTranslationMotor.setSelectedSensorPosition(0+getArmInchesEncoderTicksAbsolute(inchesOffset));
        targetInchesTicks = 0;
    }

    private void setTargetTicks(double target){
        targetInchesTicks = target;
    }

    public double getTargetInches() {
        return targetInches;
    }

    public synchronized void setTargetArmInchesPositionAbsolute(double targetInches) {
        extenderControlMode = ArmControlMode.HOLD;
        SmartDashboard.putNumber("TARGET ARM INCHES", targetInches);
        this.targetInches = targetInches;
        targetInchesTicks = getArmInchesEncoderTicksAbsolute(targetInches);
        armTranslationMotor.set(TalonFXControlMode.Position, targetInchesTicks);
    }

    public double limitArmInches(double targetInches){
        if(targetInches < Constants.ARM_EXTENDER_MIN_EXTEND_INCHES){
            return Constants.ARM_EXTENDER_MIN_EXTEND_INCHES;
        }else if(targetInches > Constants.ARM_EXTENDER_MAX_EXTEND_INCHES){
            return Constants.ARM_EXTENDER_MAX_EXTEND_INCHES;
        }

        return targetInches;   
    }

    public synchronized void setTranslationalSpeed(double speed)
    {
        setTranslationalSpeed(speed, false);
    }

    public synchronized void setTranslationalSpeed(double speed, boolean override) {
        manualTranslationSpeed = speed;
        double curSpeed = speed;

        if(!override) {
            extenderControlMode = ArmControlMode.MANUAL;
        }

        if (!override && getArmInches() < Constants.ARM_EXTENDER_MIN_EXTEND_INCHES && speed < 0.0) {
            curSpeed = 0;
        } else if (!override && getArmInches() > Constants.ARM_EXTENDER_MAX_EXTEND_INCHES && speed > 0.0) {
            curSpeed = 0;
        } else if (!override && getArmInches() - Constants.ARM_EXTENDER_MIN_EXTEND_INCHES <= 2 && curSpeed < 0) {
            curSpeed *= 0.25;
        }

        armTranslationMotor.set(ControlMode.PercentOutput, curSpeed);
    }

    public void cubeExtend(){
        if(getScoreMode()!=ScoreMode.LOW){
            double target = getArmInches()+6;
            setTargetArmInchesPositionAbsolute(target);
            // if(getScoreMode()==ScoreMode.HIGH){
            //     setArmDegreesPositionAbsolute(113);
            // }
        }else{
            double target = getArmDegreesIntegrated()+10;
            setArmDegreesPositionAbsolute(target);
        }
    }

    public synchronized void setTranslationalHold(){
        // if(firstHoldSet){
        //     firstHoldSet = false;
        //     setTargetTicks(armTranslationMotor.getSelectedSensorPosition());
        // }
        extenderControlMode = ArmControlMode.HOLD;
        armTranslationMotor.selectProfileSlot(0, 0);    

        armTranslationMotor.set(ControlMode.MotionMagic, targetInchesTicks);
    }
        //#endregion

    @Override
    public void periodic(){
        SmartDashboard.putString("score mode", Arm.getInstance().scoreMode.name());
        SmartDashboard.putString("arm control mode", rotationControlMode.name());
        SmartDashboard.putString("closest score mode", ScoreMode.getClosestMode(getArmDegreesIntegrated()).name());

        SmartDashboard.putNumber("arm degrees internal", getArmDegreesIntegrated());
        // SmartDashboard.putNumber("arm degrees ext", getArmDegreesExternal());
        SmartDashboard.putNumber("arm inches", getArmInches());

        SmartDashboard.putNumber("target arm degrees", getTargetDegrees());
        SmartDashboard.putNumber("extendo voltage", armTranslationMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("extendo current", armTranslationMotor.getStatorCurrent());

        if(rotationControlMode == ArmControlMode.MANUAL){
            targetDegreesTicks = armRotationMotor.getSelectedSensorPosition();
            setScoreMode(ScoreMode.getClosestMode(getArmDegreesIntegrated()));
        }

        if (rotationControlMode == ArmControlMode.MANUAL){
            if (getArmDegreesIntegrated() < Constants.ARM_ROTATOR_MIN_ROTATION_DEGREES && manualRotationSpeed < 0.0) {
                setRotationHold();
            } else if (getArmDegreesIntegrated() > Constants.ARM_ROTATOR_MAX_ROTATION_DEGREES && manualRotationSpeed > 0.0) {
                setRotationHold();
            }
        }

        if(extenderControlMode==ArmControlMode.MANUAL)
            setTargetTicks(armTranslationMotor.getSelectedSensorPosition());
        
        if (extenderControlMode == ArmControlMode.MANUAL) {
            if (getArmInches() < Constants.ARM_EXTENDER_MIN_EXTEND_INCHES && manualTranslationSpeed < 0.0) {
                setTranslationalHold();
            } else if (getArmInches() > Constants.ARM_EXTENDER_MAX_EXTEND_INCHES && manualTranslationSpeed > 0.0) {
                setTranslationalHold();
            }
        } 
    }
}