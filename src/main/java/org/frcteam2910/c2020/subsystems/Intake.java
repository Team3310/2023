package org.frcteam2910.c2020.subsystems;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Servo;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private int kIntakeVelocitySlot = 0;
    private int kIntakePositionSlot = 1;

    boolean hasSetIntakeZero = false;
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
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
    //#endregion
        //#region intake

        public void setControlMode(IntakeControlMode c){
            this.controlMode = c;
        }

        public void setIntakeSetZero(boolean zero){
            this.hasSetIntakeZero = zero;
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

        public DigitalInput getCubeSensor() {
            return cubeSensor;
        }

        public DigitalInput getConeSensor() {
            return coneSensor;
        }
        //#endregion
    //#endregion

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("DIO Cone", coneSensor.get());
        SmartDashboard.putBoolean("DIO Cube", cubeSensor.get());
        SmartDashboard.putBoolean("set intake zero", hasSetIntakeZero);
        SmartDashboard.putNumber("intake motor current", intakeMotor.getStatorCurrent());
    }

	public IntakeControlMode getControlMode() {
		return controlMode;
	}
}