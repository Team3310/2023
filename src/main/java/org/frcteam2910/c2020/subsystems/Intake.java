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
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    //servos
    private final Servo rightServo = new Servo(Constants.RIGHT_SERVO_PORT);
    private final Servo leftServo = new Servo(Constants.LEFT_SERVO_PORT);

    //falcons
    private TalonFX intakeMotor;

    //sensors
    private DigitalInput cubeSensor;
    private DigitalInput coneSensor;

    //conversions
    
    //misc
    private Controller secondaryController;

    boolean hasSetIntakeZero = false;
    boolean setConeIntake = false;
    boolean setCubeIntake = false;
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
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_PORT, "rio");
        intakeMotor.setInverted(true);
        leftServo.setInverted(true);

        //intakeMotor.con

        cubeSensor = new DigitalInput(1);
        coneSensor = new DigitalInput(0);
    }
    //#endregion
    
    //#region Class Methods
        //#region servo
    public void setServoPosition(double position){
        lastCommandedPosition=position;
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
    //#endregion
        //#region intake

        public boolean getSetConeIntake(){
            return setConeIntake;
        }

        public boolean getSetCubeIntake(){
            return setCubeIntake;
        }
        
        public void variableIntakeRPM(){
            hasSetIntakeZero = true;
            if(getRightTriggerAxis().getButton(0.1).getAsBoolean()){
                setRollerRPM(-getRightTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
                hasSetIntakeZero = false;
                setCubeIntake = true;
            }
            else if(getLeftTriggerAxis().getButton(0.1).getAsBoolean()){
                setRollerRPM(getLeftTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
                hasSetIntakeZero = false;
                setCubeIntake = false;
            }
            else if(RobotContainer.getInstance().getSecondaryController().getLeftBumperButton().getAsBoolean()) {
                setRollerRPM(Constants.INTAKE_SPIT_RPM);
                hasSetIntakeZero = false;
                setConeIntake = false;
            }
            else if(RobotContainer.getInstance().getSecondaryController().getRightBumperButton().getAsBoolean()) {
                setRollerRPM(Constants.INTAKE_COLLECT_RPM);
                hasSetIntakeZero = false;
                setConeIntake = true;
            }
            else{
                hasSetIntakeZero = cubeSensor.get() || coneSensor.get();
                if(hasSetIntakeZero){
                    setRollerSpeed(0);
                    setServoPosition(0.0);
                    setConeIntake = false;
                    setCubeIntake = false;
                }
                else{
                    setServoPosition(1.0);
                }
            }
        }

        public void setRollerSpeed(double speed) {
            this.intakeMotor.set(ControlMode.PercentOutput, speed);
        }
        
        public void setRollerRPM(double rpm) {
            intakeMotor.set(ControlMode.Velocity, this.RollerRPMToNativeUnits(rpm));
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

        public DigitalInput getCubeSensor() {
            return cubeSensor;
        }

        public DigitalInput getConeSensor() {
            return coneSensor;
        }

        private Axis getRightTriggerAxis(){return secondaryController.getRightTriggerAxis();}
        private Axis getLeftTriggerAxis(){return secondaryController.getLeftTriggerAxis();}
        //#endregion
    //#endregion

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("DIO Cone", coneSensor.get());
        SmartDashboard.putBoolean("DIO Cube", cubeSensor.get());
        SmartDashboard.putNumber("intake motor current", intakeMotor.getStatorCurrent());
       
        variableIntakeRPM();
    }
}