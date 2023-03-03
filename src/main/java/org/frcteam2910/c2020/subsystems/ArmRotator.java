package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.common.robot.input.Axis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmRotator implements Subsystem{
    public enum ArmRotationMode{
        MANUAL, HOLD
    }

    //falcons
    private TalonFX armRotationMotor;

    //conversions
    private double ARM_ROTATION_GEAR_RATIO = (50.0/11.0)*(60.0/26.0)*(36.0/18.0)*(36.0/18.0)*(60.0/18.0)*(2.0/3.0);
    private double ARM_REVOLUTIONS_TO_ENCODER_TICKS = ARM_ROTATION_GEAR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private double ARM_DEGREES_TO_ENCODER_TICKS = ARM_REVOLUTIONS_TO_ENCODER_TICKS/360.0;
    //misc
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

        armRotationMotor.config_kF(0, 0.005);
        armRotationMotor.config_kP(0, 0.01);
        armRotationMotor.config_kI(0, 0.0);
        armRotationMotor.config_kD(0, 0.0);
    }
    //#endregion
    
    //#region Class Methods
        //#region arm
    public void setRotationControlMode(ArmRotationMode mode){
        controlMode = mode;
    }

    public Axis getArmRotationAxis() {
        return RobotContainer.getInstance().getSecondaryController().getLeftYAxis();
    }

    public double getArmDegrees(){
        return (armRotationMotor.getSelectedSensorPosition() / ARM_DEGREES_TO_ENCODER_TICKS)*(2.0/3.0)+ degreesOffset;
    }

    public double getArmDegreesEncoderTicks(double degrees){
        return degrees * ARM_DEGREES_TO_ENCODER_TICKS;
    }

    public void setArmDegreesZero(double offset){
        degreesOffset = offset;
        targetDegreesTicks = 0;
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

    public synchronized void setArmDegreesPositionAbsolute(double degrees) {
        controlMode = ArmRotationMode.HOLD;
        armRotationMotor.selectProfileSlot(0, 0);
        targetDegreesTicks = getArmDegreesEncoderTicks(limitArmDegrees(degrees));
        armRotationMotor.set(ControlMode.Position, targetDegreesTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setRotationSpeed(double speed) {
        manualRotationSpeed = speed;
        double curSpeed = speed;

        controlMode = ArmRotationMode.MANUAL;
        if (getArmDegrees() < Constants.MIN_ARM_DEGREES && speed < 0.0) {
            curSpeed = 0;
        } else if (getArmDegrees() > Constants.MAX_ARM_DEGREES && speed > 0.0) {
            curSpeed = 0;
        }

        armRotationMotor.set(ControlMode.PercentOutput, curSpeed);
    }

    public synchronized void setRotationHold(){
        // if(firstHoldSet){
        //     firstHoldSet = false;
        //     targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(getArmDegrees()));
        // }
        controlMode = ArmRotationMode.HOLD;
        armRotationMotor.selectProfileSlot(0, 0);    
        //targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(getArmDegrees()));
        armRotationMotor.set(ControlMode.Position, targetDegreesTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    private double getTargetDegrees(){
        return targetDegreesTicks/ARM_DEGREES_TO_ENCODER_TICKS;
    }
        //#endregion
        
    //#endregion

    @Override
    public void periodic(){
        if(controlMode == ArmRotationMode.MANUAL)
            targetDegreesTicks = armRotationMotor.getSelectedSensorPosition()*(2.0/3.0);

        // SmartDashboard.putNumber("arm degress", getArmDegrees());
        // SmartDashboard.putNumber("command Arm Degrees", getTargetDegrees());
        // SmartDashboard.putNumber("rotation speed", manualRotationSpeed);
        // SmartDashboard.putString("rotator control mode", controlMode.toString());
        // SmartDashboard.putNumber("target ticks", targetDegreesTicks);
        // SmartDashboard.putNumber("rotation motor ticks", armRotationMotor.getSelectedSensorPosition());
        // SmartDashboard.putNumber("degress to ticks", ARM_DEGREES_TO_ENCODER_TICKS);
        if (controlMode == ArmRotationMode.MANUAL){
            if (getArmDegrees() < Constants.MIN_ARM_DEGREES && manualRotationSpeed < 0.0) {
                setRotationHold();
            } else if (getArmDegrees() > Constants.MAX_ARM_DEGREES && manualRotationSpeed > 0.0) {
                setRotationHold();
            }
        }
    }
}