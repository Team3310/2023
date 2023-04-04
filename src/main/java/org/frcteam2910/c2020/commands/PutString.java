package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PutString extends CommandBase {
    private String message;
    private String name;

    public PutString(String message, String name) {
        this.message = message;
        this.name = name;
    }

    @Override
    public void execute() {
        SmartDashboard.putString(name, message);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
