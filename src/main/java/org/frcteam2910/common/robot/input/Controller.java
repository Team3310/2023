package org.frcteam2910.common.robot.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * <p>An interface for easily implementing gamepads as an input source.</p>
 *
 * @author Jacob Bublitz
 * @see XboxController
 * @since 1.0
 */
public abstract class Controller extends GenericHID {
	/**
	 * Construct an instance of a device.
	 *
	 * @param port The port index on the Driver Station that the device is plugged into.
	 */
	public Controller(int port) {
		super(port);
	}

	public abstract Trigger getLeftTriggerAxis();

	public abstract Axis getLeftXAxis();

	public abstract Axis getLeftYAxis();

	public abstract Trigger getRightTriggerAxis();

	public abstract Axis getRightXAxis();

	public abstract Axis getRightYAxis();

	/**
	 * Get the A button of the controller.
	 *
	 * @return The A button
	 * @since 1.0
	 */
	public abstract Button getAButton();

	/**
	 * Get the B button of the controller.
	 *
	 * @return The B button
	 * @since 1.0
	 */
	public abstract Button getBButton();

	/**
	 * Get the X button of the controller.
	 *
	 * @return The X button
	 * @since 1.0
	 */
	public abstract Button getXButton();

	/**
	 * Get the Y button of the controller.
	 *
	 * @return The Y button
	 * @since 1.0
	 */
	public abstract Button getYButton();

	/**
	 * Get the left bumper button of the controller.
	 *
	 * @return The left bumper button
	 * @since 1.0
	 */
	public abstract Button getLeftBumperButton();

	/**
	 * Get the right bumper button of the controller.
	 *
	 * @return The right bumper button
	 * @since 1.0
	 */
	public abstract Button getRightBumperButton();

	/**
	 * Get the back button of the controller.
	 *
	 * @return The back button
	 * @since 1.0
	 */
	public abstract Button getBackButton();

	/**
	 * Get the start button of the controller.
	 *
	 * @return The start button
	 * @since 1.0
	 */
	public abstract Button getStartButton();

	/**
	 * Get the left joystick button of the controller.
	 *
	 * @return The left joystick button
	 * @since 1.0
	 */
	public abstract Button getLeftJoystickButton();

	/**
	 * Get the right joystick button of the controller.
	 *
	 * @return The right joystick button
	 * @since 1.0
	 */
	public abstract Button getRightJoystickButton();

	/**
	 * Get a D-Pad button of the controller.
	 *
	 * @param direction The direction of the D-Pad button
	 * @return The D-Pad button of the specified direction
	 * @since 1.0
	 */
	public abstract Trigger getDPadButton(DPadButton.Direction direction);
}
