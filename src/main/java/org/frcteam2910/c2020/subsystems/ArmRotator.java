package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmRotator implements Subsystem{
    public enum ArmRotationMode{
        MANUAL, HOLD
    }

    //falcons
    private TalonFX armRotationMotor;

    //conversions
    private double ARM_REVOLUTIONS_TO_ENCODER_TICKS = Constants.ARM_ROTATION_GEAR_RATIO*Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private double ARM_DEGREES_TO_ENCODER_TICKS = 2.574/Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;//ARM_REVOLUTIONS_TO_ENCODER_TICKS/360;
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

        armRotationMotor.configMotionCruiseVelocity(6000);
        armRotationMotor.configMotionAcceleration(14000);
        armRotationMotor.configMotionSCurveStrength(4);

        armRotationMotor.config_kF(0, 0.09);
        armRotationMotor.config_kP(0, 0.40);
        armRotationMotor.config_kI(0, 0.0001);
        armRotationMotor.config_kD(0, 0.0);
        
    }
    //#endregion
    
    //#region Class Methods
        //#region arm
    public void setRotationControlMode(ArmRotationMode mode){
        controlMode = mode;
    }

    public double getRotationRotations(){
        return armRotationMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / Constants.ARM_ROTATION_GEAR_RATIO;
    }

    public double getArmDegrees(){
        return (getRotationRotations() * ARM_DEGREES_TO_ENCODER_TICKS)+degreesOffset;
    }

    public double getArmDegreesEncoderTicksAbsolute(double degrees){
        return (int) (degrees * ARM_DEGREES_TO_ENCODER_TICKS);
    }

    public void setArmDegreesZero(double offset){
        degreesOffset = offset;
        armRotationMotor.setSelectedSensorPosition(0);
        targetDegreesTicks = 0;
    }

    public double limitArmDegrees(double targetDegrees){
        if(targetDegrees < Constants.MIN_ARM_DEGREES){
            return Constants.MIN_ARM_DEGREES;
        }else if(targetDegrees > Constants.MAX_ARM_DEGREES){
            return Constants.MAX_ARM_DEGREES;
        }

        return targetDegrees;   
    }

    public synchronized void setArmDegreesMotionMagicPositionAbsolute(double degrees) {
        controlMode = ArmRotationMode.HOLD;
        armRotationMotor.selectProfileSlot(0, 0);
        targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(degrees));
        armRotationMotor.set(ControlMode.Position, targetDegreesTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setRotationSpeed(double speed) {
        manualRotationSpeed = speed;
        double curSpeed = speed;

        controlMode = ArmRotationMode.MANUAL;
        // if (getArmDegrees() < Constants.MIN_ARM_DEGREES && speed < 0.0) {
        //     curSpeed = 0;
        // } else if (getArmDegrees() > Constants.MAX_ARM_DEGREES && speed > 0.0) {
        //     curSpeed = 0;
        // }

        armRotationMotor.set(ControlMode.PercentOutput, curSpeed);
    }

    public synchronized void setRotationHold(){
        if(firstHoldSet){
            firstHoldSet = false;
            targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(getArmDegrees()));
        }
        controlMode = ArmRotationMode.HOLD;
        armRotationMotor.selectProfileSlot(0, 0);    
        //targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(limitArmDegrees(getArmDegrees()));
        armRotationMotor.set(ControlMode.Position, targetDegreesTicks*2, DemandType.ArbitraryFeedForward, 0.04);
    }
        //#endregion
        
    //#endregion

    @Override
    public void periodic(){
        if(controlMode == ArmRotationMode.MANUAL)
            targetDegreesTicks = getArmDegreesEncoderTicksAbsolute(getArmDegrees());

        SmartDashboard.putNumber("arm degress", getArmDegrees());
        SmartDashboard.putNumber("command Arm Degrees", targetDegreesTicks/ARM_DEGREES_TO_ENCODER_TICKS);
        SmartDashboard.putNumber("rotation speed", manualRotationSpeed);
        SmartDashboard.putString("rotator control mode", controlMode.toString());
        // if (rotatinControlMode == ArmControlMode.MANUAL) {
        //     if (getArmDegrees() < Constants.MIN_ARM_DEGREES && manualRotationSpeed < 0.0) {
        //         setRotationHold();
        //     } else if (getArmDegrees() > Constants.MAX_ARM_DEGREES && manualRotationSpeed > 0.0) {
        //         setRotationHold();
        //     }
        // }
    }
}