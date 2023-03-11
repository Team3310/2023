package org.frcteam2910.c2020;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class Servo extends PWMMotorController {
  private static final double kMaxServoAngle = 130.0;
  private static final double kMinServoAngle = 0.0;

  protected static final double kDefaultMaxServoPWM = 2.45;
  protected static final double kDefaultMinServoPWM = 0.55;

  private boolean inverted;

  /**
   * Constructor.<br>
   *
   * <p>By default {@value #kDefaultMaxServoPWM} ms is used as the maxPWM value<br>
   * By default {@value #kDefaultMinServoPWM} ms is used as the minPWM value<br>
   */
  public Servo(final int channel) {
    super("generic", channel);
    m_pwm.setBounds(kDefaultMaxServoPWM, 1.52, 1.5, 1.48, kDefaultMinServoPWM);
    m_pwm.setPeriodMultiplier(PeriodMultiplier.k1X);
    m_pwm.setZeroLatch();
    HAL.report(tResourceType.kResourceType_Servo, getChannel() + 1);
    SendableRegistry.setName(this, "Servo", getChannel());
  }

  /**
   * Set the servo position.
   *
   * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
   *
   * @param value Position from 0.0 to 1.0.
   */
  public void set(double value) {
    int invert = inverted?-1:1;
    setPosition(value*invert);
  }
 
  public void setSpeed(double speed){
    int invert = inverted?-1:1;
    m_pwm.setSpeed(speed*invert);
  }

  public void setPosition(double position){
    int invert = inverted?-1:1;
    m_pwm.setPosition(position*invert);
  }

  public void setInverted(boolean inverted){
    this.inverted = inverted;
  }
  /**
   * Get the servo position.
   *
   * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
   * This returns the commanded position, not the position that the servo is actually at, as the
   * servo does not report its own position.
   *
   * @return Position from 0.0 to 1.0.
   */
  public double get() {
    return m_pwm.getPosition();
  }

  /**
   * Set the servo angle.
   *
   * <p>The angles are based on the HS-322HD Servo, and have a range of 0 to 180 degrees.
   *
   * <p>Servo angles that are out of the supported range of the servo simply "saturate" in that
   * direction In other words, if the servo has a range of (X degrees to Y degrees) than angles of
   * less than X result in an angle of X being set and angles of more than Y degrees result in an
   * angle of Y being set.
   *
   * @param degrees The angle in degrees to set the servo.
   */
  public void setAngle(double degrees) {
    if (degrees < kMinServoAngle) {
      degrees = kMinServoAngle;
    } else if (degrees > kMaxServoAngle) {
      degrees = kMaxServoAngle;
    }

    setPosition(((degrees - kMinServoAngle)) / getServoAngleRange());
  }

  /**
   * Get the servo angle.
   *
   * <p>This returns the commanded angle, not the angle that the servo is actually at, as the servo
   * does not report its own angle.
   *
   * @return The angle in degrees to which the servo is set.
   */
  public double getAngle() {
    return m_pwm.getPosition() * getServoAngleRange() + kMinServoAngle;
  }

  private double getServoAngleRange() {
    return kMaxServoAngle - kMinServoAngle;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Servo");
    builder.addDoubleProperty("Value", this::get, this::set);
  }
}
