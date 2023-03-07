package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.Servo;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    //servos
    private final Servo rightServo = new Servo(Constants.RIGHT_SERVO_PORT);
    private final Servo leftServo = new Servo(Constants.LEFT_SERVO_PORT);

    //Talons
    private TalonFX intakeMotor;

    //sensors
    private DigitalInput cubeSensor;
    private DigitalInput coneSensor;
    
    private Controller secondaryController;

    boolean hasSetIntakeZero = false;
    private double lastCommandedPosition;

    private static Intake INSTANCE=null;

    //#region Constructors
    public static Intake getInstance(){
        if(INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }
    
    private Intake(){
        CommandScheduler.getInstance().registerSubsystem(this);
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_PORT);
        intakeMotor.setInverted(true);
        leftServo.setInverted(true);

        cubeSensor = new DigitalInput(0);
        coneSensor = new DigitalInput(1);
    }
    //#endregion

    //#region servo
    public void setServoPosition(double position){
        lastCommandedPosition=position;
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
    //#endregion

    //#region intake
        
    public void variableIntakeRPM(){
        if(getIntakeAxis().getButton(0.1).getAsBoolean()){
            setRollerRPM(getIntakeAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
            hasSetIntakeZero = false;
        }
        else if(getOuttakeAxis().getButton(0.1).getAsBoolean()){
            setRollerRPM(-getOuttakeAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
            hasSetIntakeZero = false;
        }
        else if(RobotContainer.getInstance().getSecondaryController().getLeftBumperButton().getAsBoolean()) {
            setRollerRPM(Constants.INTAKE_SPIT_RPM);
            hasSetIntakeZero = false;
        }
        else if(RobotContainer.getInstance().getSecondaryController().getRightBumperButton().getAsBoolean()) {
            setRollerRPM(Constants.INTAKE_COLLECT_RPM);
            hasSetIntakeZero = false;
        }
        else{
            setRollerSpeed(0);
            if(hasSetIntakeZero){
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
        return (rpm * Constants.INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS) / (10*60);
    }
    public double NativeUnitsToRollerRPM(double ticks) {
        return (ticks / Constants.INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS) * (10*60);
    }
    
    public void setController(Controller secondaryController){
        this.secondaryController = secondaryController;
    }

    public Axis getIntakeAxis() {
        return secondaryController.getRightTriggerAxis();
    }

    public Axis getOuttakeAxis() {
        return secondaryController.getLeftTriggerAxis();
    }

    public DigitalInput getCubeSensor() {
        return cubeSensor;
    }
    
    public DigitalInput getConeSensor() {
        return coneSensor;
    }
    //#endregion

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("right trigger axis", getRightTriggerAxis().get());
        // SmartDashboard.putNumber("left trigger axis", getLeftTriggerAxis().get());
        // SmartDashboard.putNumber("rpm", ((60.0*(1000/100)*intakeMotor.getSelectedSensorVelocity())/2048));
        SmartDashboard.putNumber("rpm", NativeUnitsToRollerRPM(intakeMotor.getSelectedSensorVelocity()));

        variableIntakeRPM();
    }
}