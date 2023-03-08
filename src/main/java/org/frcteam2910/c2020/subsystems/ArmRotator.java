package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmRotator implements Subsystem{
    public enum ArmRotationMode{
        MANUAL, HOLD
    }

    private TalonFX armRotationMotor;

    boolean firstHoldSet = true;

    private ArmRotationMode controlMode = ArmRotationMode.MANUAL;
    private double degreesOffset;
    private double targetDegreesTicks;
    private double manualRotationSpeed;

    private static ArmRotator INSTANCE;

    //#region Constructors
    public static ArmRotator getInstance(){
        if(INSTANCE == null) {
            INSTANCE = new ArmRotator();
        }
        return INSTANCE;
    }
    
    private ArmRotator(){
        armRotationMotor = new TalonFX(Constants.ARM_ROTATION_MOTOR_PORT, "Drivetrain");

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        armRotationMotor.configAllSettings(configs);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 30.0;
        statorCurrentConfigs.enable = true;
        armRotationMotor.configStatorCurrentLimit(statorCurrentConfigs);

        armRotationMotor.setNeutralMode(NeutralMode.Brake);

        armRotationMotor.config_kF(0, 0.0);
        armRotationMotor.config_kP(0, 0.025);
        armRotationMotor.config_kI(0, 0.0000001);
        armRotationMotor.config_kD(0, 0.0);
        
    }
    //#endregion
    
    //#region Arm Methods
    public void setRotationControlMode(ArmRotationMode mode){
        controlMode = mode;
    }

    public double getArmDegrees(){
        return (armRotationMotor.getSelectedSensorPosition() / Constants.ARM_DEGREES_TO_ENCODER_TICKS)+ degreesOffset;
    }

    public double getArmDegreesEncoderTicks(double degrees){
        return degrees * Constants.ARM_DEGREES_TO_ENCODER_TICKS;
    }

    public void setArmDegreesZero(double offset){
        degreesOffset = offset;
        targetDegreesTicks = 0;
        armRotationMotor.setSelectedSensorPosition(0);
    }

    public double limitArmDegrees(double targetDegrees){
        if(targetDegrees < Constants.ARM_MIN_ROTATION_DEGREES){
            return Constants.ARM_MIN_ROTATION_DEGREES;
        }else if(targetDegrees > Constants.ARM_MAX_ROTATION_DEGREES){
            return Constants.ARM_MAX_ROTATION_DEGREES;
        }

        return targetDegrees;   
    }

    public synchronized void setArmDegreesPositionAbsolute(double degrees) {
        controlMode = ArmRotationMode.HOLD;
        armRotationMotor.selectProfileSlot(0, 0);
        targetDegreesTicks = getArmDegreesEncoderTicks(limitArmDegrees(degrees));
        // armRotationMotor.set(ControlMode.Position, targetDegreesTicks, DemandType.ArbitraryFeedForward, 0.03);
        armRotationMotor.set(ControlMode.Position, targetDegreesTicks);
    }

    public synchronized void setRotationSpeed(double speed) {
        manualRotationSpeed = speed;

        controlMode = ArmRotationMode.MANUAL;
        if (getArmDegrees() < Constants.ARM_MIN_ROTATION_DEGREES && speed < 0.0) {
            speed = 0;
        } else if (getArmDegrees() > Constants.ARM_MAX_ROTATION_DEGREES && speed > 0.0) {
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
        controlMode = ArmRotationMode.HOLD;
        armRotationMotor.selectProfileSlot(0, 0);    
        //targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(getArmDegrees()));
        armRotationMotor.set(ControlMode.Position, targetDegreesTicks);
    }

    private double getTargetDegrees(){
        return targetDegreesTicks/Constants.ARM_DEGREES_TO_ENCODER_TICKS;
    }   

    public void setMotorNeutralMode(NeutralMode nm) {
        armRotationMotor.setNeutralMode(nm);
    }
    //#endregion

    @Override
    public void periodic(){
        if(controlMode == ArmRotationMode.MANUAL)
            targetDegreesTicks = armRotationMotor.getSelectedSensorPosition();

        SmartDashboard.putNumber("Arm Degrees", getArmDegrees());
        // SmartDashboard.putString("rotator control mode", controlMode.toString());
        SmartDashboard.putNumber("Arm Target Degrees", getTargetDegrees());
        
        if (controlMode == ArmRotationMode.MANUAL){
            if (getArmDegrees() < Constants.ARM_MIN_ROTATION_DEGREES && manualRotationSpeed < 0.0) {
                setRotationHold();
            } else if (getArmDegrees() > Constants.ARM_MAX_ROTATION_DEGREES && manualRotationSpeed > 0.0) {
                setRotationHold();
            }
        }
    }
}