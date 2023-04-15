package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.awt.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.LED;

public class FlashLEDs extends CommandBase {
    private final LED led;
    private final boolean wantCone;

    private Intake intake = Intake.getInstance();

    private boolean onLED = false;
    private double lastTimeLedsSet = 0.0;

    private final Color colorToSet;

    private final int ledAmount = 8;

    public FlashLEDs(LED led, boolean wantCone) {
        this.led = led;
        this.wantCone = wantCone;
        addRequirements(this.led);

        led.wantingObject = true;
        if(wantCone) {
            // Cone
            colorToSet = new Color(252, 100, 3);
        }
        else {
            // Cube
            colorToSet = new Color(25, 0, 25);
        }
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Take (currentTime - previousTime) and see if it's greater than some duration
        if(Timer.getFPGATimestamp() - lastTimeLedsSet > 0.25) {
            // Half a second has elapsed
            if(onLED) {
                // LEDs are illuminated
                led.setLEDs(0, 0, 0, 0, ledAmount);
            }
            else {
                // LEDs are off
                led.setLEDs(colorToSet.getRed(), colorToSet.getGreen(), colorToSet.getBlue(), 0, ledAmount);
            }

            onLED = !onLED;
            lastTimeLedsSet = Timer.getFPGATimestamp();
        }
        // else leave the LEDs alone, on or off
    }

    @Override
    public boolean isFinished() {
        if(wantCone) {
            // We were told to flash for cone
            return intake.getConeSensor().get();
        }
        else {
            return intake.getCubeSensor().get();
        }
    }

    @Override
    public void end(boolean isInterrupt) {
        led.setLEDs(colorToSet.getRed(), colorToSet.getGreen(), colorToSet.getBlue(), 0,ledAmount);
        led.wantingObject = false;
    }
}