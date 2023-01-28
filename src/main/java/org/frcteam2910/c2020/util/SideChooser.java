package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SideChooser {
    private SendableChooser<sideMode> sideChooser = new SendableChooser<>();

    public SideChooser() {

        sideChooser.setDefaultOption("Blue", sideMode.BLUE);
        sideChooser.addOption("Red", sideMode.RED);
    }

    public SendableChooser<sideMode> getSideModeChooser() {
        return sideChooser;
    }

    public sideMode getSide() {
        return sideChooser.getSelected();
    }

    public enum sideMode { 
        RED,
        BLUE
    }
}
