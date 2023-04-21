package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * this class is used to check the movement of a moving object over time
 */
public class CheckMovement{
    private Timer moveTimer;
    private double lastPosition;
    private double maxTimeNoChange;
    private double minChange;
    /**
     * @param start the start position of the object you are checking the movement of
     * @param minChange the least amount of change in position required for it to be considered moving
     * @param maxTimeNoChange the amount of time that it doesn't move before you say it isn't moving
     */
    public CheckMovement(double start, double minChange, double maxTimeNoChange){
        this.lastPosition = start;
        this.minChange = minChange;
        this.maxTimeNoChange = maxTimeNoChange;
        
        moveTimer = new Timer();
        moveTimer.start();
    }

    /**
     * this is meant to be called in a command to check the movement of the object throughout the command
     * @param position the new position of the object
     * @return if the object has not moved in the amount of time specified
     */
    public boolean check(double position){
        double change = Math.abs(lastPosition-position) ;
        boolean hasMoved = change > minChange;
        this.lastPosition = position;
        if(hasMoved){
            moveTimer.reset();
            moveTimer.start();
        }
        return moveTimer.hasElapsed(maxTimeNoChange);
    }
}
