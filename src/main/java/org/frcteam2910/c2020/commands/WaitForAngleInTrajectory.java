package org.frcteam2910.c2020.commands;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitForAngleInTrajectory extends SequentialCommandGroup{
    public WaitForAngleInTrajectory(Trajectory trajectory, Rotation2 angle,Command...commands){
        double waitTime = 0.0;
        Command[] command = new Command[commands.length+1];
        for(double time=0.0; time<trajectory.getDuration(); time+=0.01){
            if(trajectory.calculate(time).getPathState().getRotation().equals(angle, 2)){
                waitTime = time;
                break;
            }
        }
        command[0] = new WaitCommand(waitTime);
        for(int i=0;i<commands.length;i++){
            command[i+1] = commands[i];
        }
        this.addCommands(command);
    }
}
