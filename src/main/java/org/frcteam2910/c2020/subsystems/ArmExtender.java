package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmExtender implements Subsystem{
    public enum ArmExtenderMode{
        MANUAL, HOLD
    }

    //falcons
    private TalonFX armTranslationMotor;

    //conversions
    private double DRUM_DIAMETER = 0.54134;
    private double TRANSLATIONAL_ROTATIONS_TO_INCHES = Math.PI * DRUM_DIAMETER;
    private double ARM_INCHES_TO_ENCODER_TICKS = Constants.ARM_TRANSLATIONAL_GEAR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION/TRANSLATIONAL_ROTATIONS_TO_INCHES;//Constants.ARM_TRANSLATIONAL_GEAR_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * TRANSLATIONAL_ROTATIONS_TO_INCHES;
    
    //misc
    private ArmExtenderMode controlMode = ArmExtenderMode.MANUAL;
    private double inchesOffset;
    private double targetInches = 0.0;
    private double targetInchesTicks = 0;
    private double manualTranslationSpeed;
    private boolean firstHoldSet = false;
    private boolean zeroing = false;

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
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        armTranslationMotor.configAllSettings(configs);
        armTranslationMotor.setNeutralMode(NeutralMode.Brake);
        armTranslationMotor.setInverted(false);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 30.0;
        statorCurrentConfigs.enable = true;
        armTranslationMotor.configStatorCurrentLimit(statorCurrentConfigs);

        armTranslationMotor.configMotionCruiseVelocity(10000);
        armTranslationMotor.configMotionAcceleration(28000);
        armTranslationMotor.configMotionSCurveStrength(4);

        armTranslationMotor.config_kF(0, 0.0);
        armTranslationMotor.config_kP(0, 0.06); //0.1
        armTranslationMotor.config_kI(0, 0.0000001);
        armTranslationMotor.config_kD(0, 0.0);
        
    }
    //#endregion
    
    //#region Class Methods
    public void setTranslationalControlMode(ArmExtenderMode mode){
        controlMode = mode;
    }

    public double getTranslationalRotations(){
        return armTranslationMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / Constants.ARM_TRANSLATIONAL_GEAR_RATIO;
    }

    public double getArmInches(){
        return (armTranslationMotor.getSelectedSensorPosition() / ARM_INCHES_TO_ENCODER_TICKS)+inchesOffset;
    }

    public double getArmInchesEncoderTicksAbsolute(double inches){
        return (inches * ARM_INCHES_TO_ENCODER_TICKS);
    }

    public void setArmInchesZero(double inchesOffset){
        this.inchesOffset = inchesOffset;
        armTranslationMotor.setSelectedSensorPosition(0);
        targetInchesTicks = 0;
    }

    private void setTargetTicks(double target){
        targetInchesTicks = target;
    }

    public double getTargetInches() {
        return targetInches;
    }

    public boolean getArmAtZero(){
        return Math.abs(getArmInches()-Math.copySign(0.5, getArmInches()))<0.5;
    }

    public boolean getArmWithinTarget(double tolorance){
        return Math.abs(getArmInches()-Math.abs(getTargetInches())) < tolorance;
    }

    public void setTargetArmInchesPositionAbsolute(double targetInches) {
        controlMode = ArmExtenderMode.HOLD;
        this.targetInches = targetInches;
        targetInchesTicks = getArmInchesEncoderTicksAbsolute(targetInches);
        //armTranslationMotor.set(TalonFXControlMode.Position, targetInchesTicks);
        armTranslationMotor.set(TalonFXControlMode.MotionMagic, targetInchesTicks);
    }

    public double limitArmInches(double targetInches){
        if(targetInches < Constants.ARM_MIN_EXTEND_INCHES){
            return Constants.ARM_MIN_EXTEND_INCHES;
        }else if(targetInches > Constants.ARM_MAX_EXTEND_INCHES){
            return Constants.ARM_MAX_EXTEND_INCHES;
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
        if(!firstHoldSet)
            firstHoldSet=true;

        if(!override) {
            controlMode = ArmExtenderMode.MANUAL;
        }

        if (!override && getArmInches() < Constants.ARM_MIN_EXTEND_INCHES && speed < 0.0) {
            curSpeed = 0;
        } else if (!override && getArmInches() > Constants.ARM_MAX_EXTEND_INCHES && speed > 0.0) {
            curSpeed = 0;
        } else if (!override && getArmInches() - Constants.ARM_MIN_EXTEND_INCHES <= 2) {
            curSpeed *= 0.25;
        }

        armTranslationMotor.set(ControlMode.PercentOutput, curSpeed);
    }

    public void setExtenderHold(){
        setTargetArmInchesPositionAbsolute(getArmInches());
    }
        
    //#endregion

    @Override
    public void periodic(){
        if(controlMode==ArmExtenderMode.MANUAL)
            setTargetTicks(armTranslationMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("arm inches", getArmInches());
        SmartDashboard.putNumber("arm target inches", getTargetInches());
        SmartDashboard.putBoolean("is at zero", getArmAtZero());
        // SmartDashboard.putString("extender control mode", controlMode.toString());
        if (controlMode == ArmExtenderMode.MANUAL) {
            if (getArmInches() < Constants.ARM_MIN_EXTEND_INCHES && manualTranslationSpeed < 0.0) {
                setTargetArmInchesPositionAbsolute(getArmInches());
            } else if (getArmInches() > Constants.ARM_MAX_EXTEND_INCHES && manualTranslationSpeed > 0.0) {
                setTargetArmInchesPositionAbsolute(getArmInches());
            }
        } 
    }
}