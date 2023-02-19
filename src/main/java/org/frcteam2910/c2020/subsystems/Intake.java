package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Servo;
import org.frcteam2910.common.robot.input.XboxController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    public enum ArmControlMode{
        MANUAL, MOTION_MAGIC
    }
    public enum ServoControlMode{
        MANUAL, HOLD
    }
    //servos
    private final Servo rightServo = new Servo(Constants.RIGHT_SERVO_PORT);
    private final Servo leftServo = new Servo(Constants.LEFT_SERVO_PORT);

    //falcons
    private TalonFX armRotationMotor;
    private TalonFX armTranslationMotor;

    //conversions
    private double ARM_REVOLUTIONS_TO_ENCODER_TICKS = Constants.ARM_ROTATION_GEAR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private double ARM_DEGREES_TO_ENCODER_TICKS = ARM_REVOLUTIONS_TO_ENCODER_TICKS/360;
    private double DRUM_DIAMETER = 0.75;
    private double TRANSLATIONAL_ROTATIONS_TO_INCHES = Math.PI * DRUM_DIAMETER;
    private double ARM_INCHES_TO_ENCODER_TICKS = Constants.ARM_TRANSLATIONAL_GEAR_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / TRANSLATIONAL_ROTATIONS_TO_INCHES;
    
    //misc
    private ArmControlMode rotatinControlMode = ArmControlMode.MANUAL;
    private ArmControlMode translationControlMode = ArmControlMode.MANUAL;
    private ServoControlMode servoControlMode = ServoControlMode.MANUAL;
    private double inchesOffset;
    private double degreesOffset;
    private double targetDegreesTicks;
    private double targetInchesTicks;
    private double manualTranslationSpeed;
    private double manualRotationSpeed;
    private double lastCommandedSpeed;

    private static Intake INSTANCE;

    //#region Constructors
    public static Intake getInstance(){
        if(INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }
    
    private Intake(){
        armRotationMotor = new TalonFX(Constants.ARM_ROTATION_MOTOR_PORT, "Drivetrain");
        armTranslationMotor = new TalonFX(Constants.ARM_TRANSLATIONAL_MOTOR_PORT, "Drivetrain");

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        armRotationMotor.configAllSettings(configs);
        armTranslationMotor.configAllSettings(configs);

        armRotationMotor.configMotionCruiseVelocity(6000);
        armRotationMotor.configMotionAcceleration(14000);
        armRotationMotor.configMotionSCurveStrength(4);

        armTranslationMotor.configMotionCruiseVelocity(6000);
        armRotationMotor.configMotionAcceleration(14000);
        armRotationMotor.configMotionSCurveStrength(4);
        
        leftServo.setInverted(false);
        rightServo.setInverted(false);
    }
    //#endregion
    
    //#region Class Methods
        //#region servo
    public void setServoSpeed(double speed){
        lastCommandedSpeed=speed;
        leftServo.setSpeed(-speed);
        rightServo.setSpeed(speed);
    }

    public void setServoControlMode(ServoControlMode mode){
        servoControlMode=mode;
    }
    //#endregion
        //#region arm
    public void setRotationControlMode(ArmControlMode mode){
        rotatinControlMode = mode;
    }

    public void setTranslationalControlMode(ArmControlMode mode){
        translationControlMode = mode;
    }

    public double getRotationRotations(){
        return armRotationMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / Constants.ARM_ROTATION_GEAR_RATIO;
    }

    public double getArmDegrees(){
        return (getRotationRotations() * ARM_DEGREES_TO_ENCODER_TICKS)+degreesOffset;
    }

    public double getTranslationalRotations(){
        return armTranslationMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / Constants.ARM_TRANSLATIONAL_GEAR_RATIO;
    }

    public double getArmInches(){
        return (getTranslationalRotations() * ARM_INCHES_TO_ENCODER_TICKS)+inchesOffset;
    }

    public double getArmDegreesEncoderTicksAbsolute(double degrees){
        return (int) (degrees * ARM_DEGREES_TO_ENCODER_TICKS);
    }

    public double getArmInchesEncoderTicksAbsolute(double inches){
        return (int) (inches * ARM_INCHES_TO_ENCODER_TICKS);
    }

    public void setArmInchesZero(double offset){
        inchesOffset = offset;
        armTranslationMotor.setSelectedSensorPosition(0);
    }

    public void setArmDegreesZero(double offset){
        degreesOffset = offset;
        armRotationMotor.setSelectedSensorPosition(0);
    }

    public double limitArmDegrees(double targetDegrees){
        if(targetDegrees < Constants.MIN_ARM_DEGREES){
            return Constants.MIN_ARM_DEGREES;
        }else if(targetDegrees > Constants.MAX_ARM_DEGREES){
            return Constants.MAX_ARM_DEGREES;
        }

        return targetDegrees;   
    }

    public double limitArmInches(double targetInches){
        if(targetInches < Constants.MIN_ARM_INCHES){
            return Constants.MIN_ARM_INCHES;
        }else if(targetInches > Constants.MAX_ARM_INCHES){
            return Constants.MAX_ARM_INCHES;
        }

        return targetInches;   
    }

    public synchronized void setArmDegreesMotionMagicPositionAbsolute(double degrees) {
        rotatinControlMode = ArmControlMode.MOTION_MAGIC;
        armRotationMotor.selectProfileSlot(1, 0);
        targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(degrees));
        armRotationMotor.set(ControlMode.MotionMagic, targetDegreesTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setArmInchesMotionMagicPositionAbsolute(double inches) {
        rotatinControlMode = ArmControlMode.MOTION_MAGIC;
        armRotationMotor.selectProfileSlot(1, 0);
        targetInchesTicks = getArmInchesEncoderTicksAbsolute(limitArmInches(inches));
        armTranslationMotor.set(ControlMode.MotionMagic, targetInchesTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setRotationSpeed(double speed) {
        manualRotationSpeed = speed;
        double curSpeed = speed;

        rotatinControlMode = ArmControlMode.MANUAL;
        if (getArmDegrees() < Constants.MIN_ARM_DEGREES && speed < 0.0) {
            curSpeed = 0;
        } else if (getArmDegrees() > Constants.MAX_ARM_DEGREES && speed > 0.0) {
            curSpeed = 0;
        }

        armRotationMotor.set(ControlMode.PercentOutput, curSpeed);
    }

    public synchronized void setTranslationalSpeed(double speed) {
        manualTranslationSpeed = speed;
        double curSpeed = speed;

        translationControlMode = ArmControlMode.MANUAL;
        if (getArmInches() < Constants.MIN_ARM_INCHES && speed < 0.0) {
            curSpeed = 0;
        } else if (getArmInches() > Constants.MAX_ARM_INCHES && speed > 0.0) {
            curSpeed = 0;
        }

        armTranslationMotor.set(ControlMode.PercentOutput, curSpeed);
    }

    public synchronized void setRotationHold(){
        setArmDegreesMotionMagicPositionAbsolute(getArmDegrees());
    }

    public synchronized void setTranslationalHold(){
        setArmInchesMotionMagicPositionAbsolute(getArmInches());
    }
        //#endregion
    //#endregion

    @Override
    public void periodic(){
        if (rotatinControlMode == ArmControlMode.MANUAL) {
            if (getArmDegrees() < Constants.MIN_ARM_DEGREES && manualRotationSpeed < 0.0) {
                setRotationHold();
            } else if (getArmDegrees() > Constants.MAX_ARM_DEGREES && manualRotationSpeed > 0.0) {
                setRotationHold();
            }
        }
        if (translationControlMode == ArmControlMode.MANUAL) {
            if (getArmInches() < Constants.MIN_ARM_INCHES && manualTranslationSpeed < 0.0) {
                setTranslationalHold();
            } else if (getArmDegrees() > Constants.MAX_ARM_INCHES && manualTranslationSpeed > 0.0) {
                setTranslationalHold();
            }
        }

        if(servoControlMode == ServoControlMode.HOLD){
            leftServo.setSpeed(lastCommandedSpeed);
            rightServo.setSpeed(lastCommandedSpeed); 
        }  
    }
}