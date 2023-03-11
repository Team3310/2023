package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Robot;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.Servo;
import org.frcteam2910.c2020.commands.setArmSafe;
import org.frcteam2910.c2020.util.ScoreMode;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    public enum IntakeControlMode{
        HOLD, MANUAL
    }

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
    private int kIntakeVelocitySlot = 0;
    private int kIntakePositionSlot = 1;

    boolean hasSetIntakeZero = false;
    boolean setConeIntake = false;
    boolean setCubeIntake = false;
    private double lastCommandedPosition;
    private IntakeControlMode controlMode = IntakeControlMode.MANUAL;

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
        leftServo.setInverted(false);
        rightServo.setInverted(true);

        intakeMotor.config_kF(kIntakeVelocitySlot, 0.0);
        intakeMotor.config_kP(kIntakeVelocitySlot, 0.10);
        intakeMotor.config_kI(kIntakeVelocitySlot, 0.0001);
        intakeMotor.config_kD(kIntakeVelocitySlot, 0.0);
        intakeMotor.config_IntegralZone(kIntakeVelocitySlot, (int)this.RollerRPMToNativeUnits(200));

        intakeMotor.config_kF(kIntakePositionSlot, 0.0);
        intakeMotor.config_kP(kIntakePositionSlot, 0.10);
        intakeMotor.config_kI(kIntakePositionSlot, 0.0001);
        intakeMotor.config_kD(kIntakePositionSlot, 0.0);

        cubeSensor = new DigitalInput(1);
        coneSensor = new DigitalInput(0);
    }
    //#endregion
    
    //#region Class Methods
        //#region servo
    public void setServoPosition(double position){
        lastCommandedPosition = position;
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
            if(!cubeSensor.get()){
                controlMode = IntakeControlMode.MANUAL;
                hasSetIntakeZero = true;
                if(getRightTriggerAxis().getButton(0.1).getAsBoolean()){
                    if(Robot.isCompetitionBot()) {
                        // if(Arm.getInstance().getScoreMode() != ScoreMode.CONE_INTAKE)
                        //     RobotContainer.getInstance().runCommand(new SequentialCommandGroup(
                        //         new InstantCommand(() -> setServoPosition(-1.0)),
                        //         new setArmSafe(Arm.getInstance(), ScoreMode.CUBE_INTAKE)
                        //     )
                        // );
                    }
                    
                    //setRollerRPM(-getRightTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM * 2);
                    setRollerSpeed(-1.0);
                    hasSetIntakeZero = false;
                    setCubeIntake = true;
                }
                else if(getLeftTriggerAxis().getButton(0.1).getAsBoolean()){
                    //setRollerRPM(getLeftTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM * 2);
                    setRollerSpeed(1.0);
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
                    if(hasSetIntakeZero){
                        if(Robot.isCompetitionBot()) {
                            // if(Arm.getInstance().getScoreMode() != ScoreMode.ZERO)
                            //     RobotContainer.getInstance().runCommand(new SequentialCommandGroup(
                            //         new setArmSafe(Arm.getInstance(), ScoreMode.ZERO),
                            //         new InstantCommand(() -> setServoPosition(-1.0))
                            //     ) );
                        }
                        setRollerSpeed(0);
                        setConeIntake = false;
                        setCubeIntake = false;
                    }
                }
            }
            else{
                if(getLeftTriggerAxis().getButton(0.1).getAsBoolean()){
                    setRollerRPM(getLeftTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
                    hasSetIntakeZero = false;
                    setCubeIntake = false;
                }
                else if(controlMode != IntakeControlMode.HOLD){
                    setIntakeHold();
                }    
            }
        }

        public void setIntakeHold(){
            controlMode = IntakeControlMode.HOLD;
            intakeMotor.selectProfileSlot(kIntakePositionSlot, 0);
            intakeMotor.set(TalonFXControlMode.Position, intakeMotor.getSelectedSensorPosition());
        }

        public void setRollerSpeed(double speed) {
            controlMode = IntakeControlMode.MANUAL;
            this.intakeMotor.set(ControlMode.PercentOutput, speed);
        }
        
        public void setRollerRPM(double rpm) {
            controlMode = IntakeControlMode.MANUAL;
            intakeMotor.selectProfileSlot(kIntakeVelocitySlot, 0);
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
        SmartDashboard.putBoolean("set intake zero", hasSetIntakeZero);
        SmartDashboard.putNumber("intake motor current", intakeMotor.getStatorCurrent());
       
        variableIntakeRPM();
    }
}