package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class GyroAutoChooser {
    private SendableChooser<Mode> modeChooser = new SendableChooser<>();

    public GyroAutoChooser() {
        modeChooser.setDefaultOption(Mode.On.name(), Mode.On);
        modeChooser.addOption(Mode.Off.name(), Mode.Off);
    }

    public SendableChooser<Mode> getSendableChooser() {
        return modeChooser;
    }

    public Mode getMode() {
        return modeChooser.getSelected();
    }

    public enum Mode { 
        On("On"),
        Off("Off");

        Mode(String s){
            mode=s;
        }

        String mode;
        public String toString(){
            return mode;
        }
    }
}
