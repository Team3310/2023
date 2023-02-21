package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Servo;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;
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
    private TalonFX intakeMotor;

    //conversions
    private static final double INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO = 60.0 / 16.0;
    public static final double INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS = INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    
    //misc
    private Controller secondaryController;

    boolean hasSetIntakeZero = false;
    private ServoControlMode servoControlMode = ServoControlMode.MANUAL;
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
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_PORT);
        
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
        
        //#region intake
        public void variableIntakeRPM(){


            if(getRightTriggerAxis().getButton(0.1).getAsBoolean()){
                setRollerSpeed(getRightTriggerAxis().get(true));
                //setRollerRPM(getRightTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
                hasSetIntakeZero = false;
            }
            else if(getLeftTriggerAxis().getButton(0.1).getAsBoolean()){
                setRollerSpeed(-getLeftTriggerAxis().get(true));
                //setRollerRPM( -getLeftTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
                hasSetIntakeZero = false;
            }
            else{
                if(!hasSetIntakeZero){
                    setRollerSpeed(0);
                    hasSetIntakeZero = true;
                }
            }
        }

        public void setRollerSpeed(double speed) {
            this.intakeMotor.set(ControlMode.PercentOutput, speed);
            //System.out.println("Set Intake Speed = " + speed);
        }
        
        public void setRollerRPM(double rpm) {
            this.intakeMotor.set(ControlMode.Velocity, this.RollerRPMToNativeUnits(rpm));
            //System.out.println("Set Intake RPM = " + rpm);
    
        }

        public double RollerRPMToNativeUnits(double rpm) {
            return rpm * INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
        }

        public void setController(Controller secondaryController){
            this.secondaryController = secondaryController;
        }

        private Axis getRightTriggerAxis(){return secondaryController.getRightTriggerAxis();}
        private Axis getLeftTriggerAxis(){return secondaryController.getLeftTriggerAxis();}
        //#endregion
    //#endregion

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("set zero intake rpm", hasSetIntakeZero);
        SmartDashboard.putNumber("controller right axis", getRightTriggerAxis().get());

        variableIntakeRPM();
        if(servoControlMode == ServoControlMode.HOLD){
            leftServo.setSpeed(lastCommandedSpeed);
            rightServo.setSpeed(lastCommandedSpeed); 
        }  
    }
}