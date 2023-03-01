package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    private double DRUM_DIAMETER = 0.75;
    private double TRANSLATIONAL_ROTATIONS_TO_INCHES = Math.PI * DRUM_DIAMETER;
    private double ARM_INCHES_TO_ENCODER_TICKS = Constants.ARM_TRANSLATIONAL_GEAR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION/TRANSLATIONAL_ROTATIONS_TO_INCHES;//Constants.ARM_TRANSLATIONAL_GEAR_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * TRANSLATIONAL_ROTATIONS_TO_INCHES;
    
    //misc
    private ArmExtenderMode controlMode = ArmExtenderMode.MANUAL;
    private double inchesOffset;
    private double targetInchesTicks=0;
    private double manualTranslationSpeed;
    private boolean firstHoldSet = false;

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
        armTranslationMotor.setInverted(true);

        armTranslationMotor.config_kF(0, 0.0);
        armTranslationMotor.config_kP(0, 0.05); //0.1
        armTranslationMotor.config_kI(0, 0.0);
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
        return (armTranslationMotor.getSelectedSensorPosition() / ARM_INCHES_TO_ENCODER_TICKS)+inchesOffset;
    }

    public double getArmInchesEncoderTicksAbsolute(double inches){
        return (int) (inches * ARM_INCHES_TO_ENCODER_TICKS);
    }

    public void setArmInchesZero(double offset){
        inchesOffset = offset;
        armTranslationMotor.setSelectedSensorPosition(0);
        targetInchesTicks = 0;
    }

    public void setTargetTicks(double target){
        targetInchesTicks = target;
    }


    public double limitArmInches(double targetInches){
        if(targetInches < Constants.MIN_ARM_INCHES){
            return Constants.MIN_ARM_INCHES;
        }else if(targetInches > Constants.MAX_ARM_INCHES){
            return Constants.MAX_ARM_INCHES;
        }

        return targetInches;   
    }

    public synchronized void setTranslationalSpeed(double speed) {
        manualTranslationSpeed = speed;
        double curSpeed = speed;
        if(!firstHoldSet)
            firstHoldSet=true;

        controlMode = ArmExtenderMode.MANUAL;

        if (getArmInches() < Constants.MIN_ARM_INCHES && speed < 0.0) {
            curSpeed = 0;
        } else if (getArmInches() > Constants.MAX_ARM_INCHES && speed > 0.0) {
            curSpeed = 0;
        }
        armTranslationMotor.set(ControlMode.PercentOutput, curSpeed);
    }

    public synchronized void setTranslationalHold(){
        // if(firstHoldSet){
        //     firstHoldSet = false;
        //     setTargetTicks(armTranslationMotor.getSelectedSensorPosition());
        // }
        controlMode = ArmExtenderMode.HOLD;
        armTranslationMotor.selectProfileSlot(0, 0);    

        armTranslationMotor.set(ControlMode.Position, targetInchesTicks, DemandType.ArbitraryFeedForward, 0.04);
    }
        //#endregion
        
    //#endregion

    @Override
    public void periodic(){
        if(controlMode==ArmExtenderMode.MANUAL)
            setTargetTicks(armTranslationMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("arm inches", getArmInches());
        SmartDashboard.putNumber("extender speed", manualTranslationSpeed);
        SmartDashboard.putString("extender control mode", controlMode.toString());
        SmartDashboard.putNumber("commanded arm inches ticks", targetInchesTicks);
        SmartDashboard.putNumber("arm inches ticks", armTranslationMotor.getSelectedSensorPosition());
        if (controlMode == ArmExtenderMode.MANUAL) {
            if (getArmInches() < Constants.MIN_ARM_INCHES && manualTranslationSpeed < 0.0) {
                setTranslationalHold();
            } else if (getArmInches() > Constants.MAX_ARM_INCHES && manualTranslationSpeed > 0.0) {
                setTranslationalHold();
            }
        } 
    }
}