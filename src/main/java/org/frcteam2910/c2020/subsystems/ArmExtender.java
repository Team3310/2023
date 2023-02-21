package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmExtender implements Subsystem{
    public enum ArmExtenderMode{
        MANUAL, MOTION_MAGIC
    }

    //falcons
    private TalonFX armTranslationMotor;

    //conversions
    private double DRUM_DIAMETER = 0.75;
    private double TRANSLATIONAL_ROTATIONS_TO_INCHES = Math.PI * DRUM_DIAMETER;
    private double ARM_INCHES_TO_ENCODER_TICKS = ((11/60)*Math.PI*DRUM_DIAMETER)/Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;//Constants.ARM_TRANSLATIONAL_GEAR_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / TRANSLATIONAL_ROTATIONS_TO_INCHES;
    
    //misc
    private ArmExtenderMode controlMode = ArmExtenderMode.MANUAL;
    private double inchesOffset;
    private double targetInchesTicks;
    private double manualTranslationSpeed;

    private static ArmExtender INSTANCE;

    //#region Constructors
    public static ArmExtender getInstance(){
        if(INSTANCE == null) {
            INSTANCE = new ArmExtender();
        }
        return INSTANCE;
    }
    
    private ArmExtender(){
        armTranslationMotor = new TalonFX(Constants.ARM_TRANSLATIONAL_MOTOR_PORT);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;;
        armTranslationMotor.configAllSettings(configs);

        armTranslationMotor.configMotionCruiseVelocity(6000);
        armTranslationMotor.configMotionAcceleration(14000);
        armTranslationMotor.configMotionSCurveStrength(4);

        armTranslationMotor.config_kF(0, 0.055);
        armTranslationMotor.config_kP(0, 0.20); //0.1
        armTranslationMotor.config_kI(0, 0.0001);
        armTranslationMotor.config_kD(0, 0.0);
        
    }
    //#endregion
    
    //#region Class Methods
        //#region arm

    public void setTranslationalControlMode(ArmExtenderMode mode){
        controlMode = mode;
    }

    public double getTranslationalRotations(){
        return armTranslationMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / Constants.ARM_TRANSLATIONAL_GEAR_RATIO;
    }

    public double getArmInches(){
        return (getTranslationalRotations() * ARM_INCHES_TO_ENCODER_TICKS)+inchesOffset;
    }

    public double getArmInchesEncoderTicksAbsolute(double inches){
        return (int) (inches * ARM_INCHES_TO_ENCODER_TICKS);
    }

    public void setArmInchesZero(double offset){
        inchesOffset = offset;
        armTranslationMotor.setSelectedSensorPosition(0);
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

    public synchronized void setArmInchesMotionMagicPositionAbsolute(double inches) {
        controlMode = ArmExtenderMode.MOTION_MAGIC;
        armTranslationMotor.selectProfileSlot(1, 0);
        targetInchesTicks = getArmInchesEncoderTicksAbsolute(limitArmInches(inches));
        armTranslationMotor.set(ControlMode.MotionMagic, targetInchesTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setTranslationalSpeed(double speed) {
        manualTranslationSpeed = speed;
        double curSpeed = speed;

        controlMode = ArmExtenderMode.MANUAL;
        if (getArmInches() < Constants.MIN_ARM_INCHES && speed < 0.0) {
            curSpeed = 0;
        } else if (getArmInches() > Constants.MAX_ARM_INCHES && speed > 0.0) {
            curSpeed = 0;
        }

        armTranslationMotor.set(ControlMode.PercentOutput, curSpeed);
    }

    public synchronized void setTranslationalHold(){
        setArmInchesMotionMagicPositionAbsolute(getArmInches());
    }
        //#endregion
        
    //#endregion

    @Override
    public void periodic(){
        SmartDashboard.putNumber("arm inches", getArmInches());
        SmartDashboard.putNumber("extender speed", manualTranslationSpeed);
        SmartDashboard.putString("extender control mode", controlMode.toString());
        // if (translationControlMode == ArmControlMode.MANUAL) {
        //     if (getArmInches() < Constants.MIN_ARM_INCHES && manualTranslationSpeed < 0.0) {
        //         setTranslationalHold();
        //     } else if (getArmDegrees() > Constants.MAX_ARM_INCHES && manualTranslationSpeed > 0.0) {
        //         setTranslationalHold();
        //     }
        // } 
    }
}