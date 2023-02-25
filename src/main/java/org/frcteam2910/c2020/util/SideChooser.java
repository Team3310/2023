package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SideChooser {
    private SendableChooser<SideMode> sideChooser = new SendableChooser<>();

    public SideChooser() {

        sideChooser.setDefaultOption("Blue", SideMode.BLUE);
        sideChooser.addOption("Red", SideMode.RED);
    }

    public SendableChooser<SideMode> getSendableChooser() {
        return sideChooser;
    }

    public SideMode getSide() {
        return sideChooser.getSelected();
    }

    public enum SideMode { 
        RED("Red"),
        BLUE("Blue");

        SideMode(String s){
            side=s;
        }

        String side;
        public String toString(){
            return side;
        }
    }
}
