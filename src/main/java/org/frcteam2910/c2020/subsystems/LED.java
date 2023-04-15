// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.led.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LED extends SubsystemBase {
  private CANdle led = new CANdle(0, "Drivetrain");

  private static LED instance = null;

  public LED() {
    CANdleConfiguration config = new CANdleConfiguration();
    led.configAllSettings(config);
    led.setLEDs(0, 0, 0);
  }

  public void setShooterStatusLEDs() {
    int r = 0, g = 0, b = 0;
    led.setLEDs(r, g, b);
  }

  public void setLEDs(int r, int g, int b, int start, int end) {
    led.setLEDs(r, g, b, 30, start, end - start);
  }

  public static LED getInstance() {
    if (instance == null) {
      instance = new LED();
    }

    return instance;
  }

  int rgbV = 0;
  boolean cUp = true;
  int chngClr = 0;

  public boolean wantingObject = false;

  @Override
  public void periodic() {
    // setShooterStatusLEDs();
    // if(cUp) {
    //   // Counting up
    //   if(rgbV >= 255) {
    //     // Reached cap
    //     rgbV = 255;
    //     cUp = false;
    //   }
  
    //   else
    //     // Didn't reach cap, count up still
    //     rgbV++;
    // }

    // else{
    //   // Counting down
    //   if(rgbV <= 0){
    //     // Reached lower boundary
    //     rgbV = 0;
    //     cUp = true;
    //     chngClr++;
    //     // Is it time to cycle back to red if we're on blue?
    //     if(chngClr == 3){
    //       chngClr = 0;
    //     }
    //   }

    //   else rgbV--;
    // }
      
    //   if(chngClr == 0) {
    //     // Red
    //     setLEDs(rgbV, 0, 0, 0, 8);
    //   }

    //   else if(chngClr == 1){ 
    //     //Green
    //   setLEDs(0, rgbV, 0, 0, 8);
    //   }

    //   else if(chngClr == 2){
    //     //Blue 
    //   setLEDs(0, 0, rgbV, 0, 8);
    //   }
      
      if(!wantingObject && !Intake.getInstance().getCubeSensor().get() && !Intake.getInstance().getConeSensor().get())
      {
        setLEDs(0, 0, 0, 0, 8);
      }
      SmartDashboard.putBoolean("DIO CONE", Intake.getInstance().getConeSensor().get());
      SmartDashboard.putBoolean("DIO CUBE", Intake.getInstance().getCubeSensor().get());
  }
}