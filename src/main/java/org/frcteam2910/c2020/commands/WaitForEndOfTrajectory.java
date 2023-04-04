package org.frcteam2910.c2020.commands;

import org.frcteam2910.common.control.Trajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitForEndOfTrajectory extends SequentialCommandGroup{
    public WaitForEndOfTrajectory(Trajectory trajectory, double secondsBeforeEnd,Command...commands){
        Command[] command = new Command[commands.length+1];
        command[0] = new WaitCommand(trajectory.getDuration()-secondsBeforeEnd);
        for(int i=0;i<commands.length;i++){
            command[i+1] = commands[i];
        }
        this.addCommands(command);
    }
}
