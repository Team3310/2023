// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam2910.c2020.subsystems;


import com.ctre.phoenix.led.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class LED extends SubsystemBase{
  private CANdle led = new CANdle(0);

  private static LED instance = null;

  public LED() {
    CANdleConfiguration config = new CANdleConfiguration();
    led.configAllSettings(config);
    led.setLEDs(0, 0, 0);
  }

  public void setShooterStatusLEDs(){
    int r=0, g=0, b=0;
    led.setLEDs(r, g, b);
  }

  public void setLEDs(int r, int g, int b, int start, int end){
    led.setLEDs(r, g, b, 255, start, end-start);
  }

  public static LED getInstance() {
    if(instance == null) {
      instance = new LED();
    }

    return instance;
  }

  @Override
  public void periodic() {
    setShooterStatusLEDs(); 
  }
}