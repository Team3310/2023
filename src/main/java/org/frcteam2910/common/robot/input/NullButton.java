package org.frcteam2910.common.robot.input;

import edu.wpi.first.wpilibj2.command.button.InternalButton;

public final class NullButton extends InternalButton {
	private boolean value;

	public NullButton() {
		this(false);
	}

	public NullButton(boolean initialValue) {
		value = initialValue;
	}

	@Override
	public boolean getAsBoolean() {
		return value;
	}

	public void set(boolean value) {
		this.value = value;
	}
}
