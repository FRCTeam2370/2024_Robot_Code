// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  public static AddressableLED blinkahs/*In Boston accent/because were going to Massachusetts*/ = new AddressableLED(9);
  private static AddressableLEDBuffer blinkahsBuffer = new AddressableLEDBuffer(42);
  private static Optional<Alliance> alliance = DriverStation.getAlliance();
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    startLEDs();
  }


  public static void startLEDs(){
    // tells the LEDs how long they are
    blinkahs.setLength(blinkahsBuffer.getLength());
    blinkahs.start();
  }

  public static void youGotTheThing(){

    for(var i = 0; i < blinkahsBuffer.getLength(); i++){
      //good orange
      blinkahsBuffer.setRGB(i, 252, 177, 3);
    }
    
    blinkahs.setData(blinkahsBuffer);
  }


  public static void setGreen(){
    for(var i = 0; i < blinkahsBuffer.getLength()-21; i++){
      // roughly Ibots green
      blinkahsBuffer.setRGB(i, 50, 255, 0);
    }
    
    blinkahs.setData(blinkahsBuffer);
  }

  public static void setRed(){
    for(var i = 0; i < blinkahsBuffer.getLength()-21; i++){
      blinkahsBuffer.setRGB(i, 255, 0, 0);
    }
    
    blinkahs.setData(blinkahsBuffer);
  }

  public static void turnOff(){
    for(var i = 0; i < blinkahsBuffer.getLength(); i++){
      blinkahsBuffer.setRGB(i, 0, 0, 0);
    }
    
    blinkahs.setData(blinkahsBuffer);
  }


  public static boolean isBlue(){
    return alliance.get() == DriverStation.Alliance.Blue;
  }

  public static void setBlue(){
    for(var i = 21; i < blinkahsBuffer.getLength(); i++){
      blinkahsBuffer.setRGB(i, 20, 0, 255);
    }
    
    blinkahs.setData(blinkahsBuffer);
  }

  public static void setPink(){
    for(var i = 21; i < blinkahsBuffer.getLength(); i++){
      blinkahsBuffer.setRGB(i, 209, 2, 140);
    }
    
    blinkahs.setData(blinkahsBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
