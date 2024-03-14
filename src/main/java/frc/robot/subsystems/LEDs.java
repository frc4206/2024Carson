// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicTreeUI.TreeCancelEditingAction;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private AddressableLED ledstrip;
  private AddressableLEDBuffer ledBuffer;
  boolean ledInit = false;
  double initTime = 0;
  double currTime = 0;
  double flashCycleTime = .1;

  public LEDs() {
    ledstrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(44);
    ledstrip.setBitTiming(400, 850, 800, 450);
    ledstrip.setLength(ledBuffer.getLength());
    ledstrip.setData(ledBuffer);
    ledstrip.start();
  }

  public void setRed(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    ledstrip.setData(ledBuffer);
  }
  public void setRed(int offFromMax){
    for (var i = 0; i < ledBuffer.getLength() - offFromMax; i++) {
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    ledstrip.setData(ledBuffer);
  }

  public void setGreen(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    ledstrip.setData(ledBuffer);
  }
  public void setGreen(int offFromMax){
    for (var i = 0; i < ledBuffer.getLength() - offFromMax; i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    ledstrip.setData(ledBuffer);
  }
  

  public void setBlue(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 255);
    }
    ledstrip.setData(ledBuffer);
  }

  public void setWhite(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 100, 100, 100);
    }
    ledstrip.setData(ledBuffer);
  }
  public void setWhite(int offFromMax){
    for (var i = 0; i < ledBuffer.getLength() - offFromMax; i++) {
      ledBuffer.setRGB(i, 100, 100, 100);
    }
    ledstrip.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    if (SmartDashboard.getBoolean("beam broken", false)) {
      if (!ledInit) {
        ledInit = true;
        initTime = Timer.getFPGATimestamp();
      }
      currTime = Timer.getFPGATimestamp() - initTime;

      if (currTime < flashCycleTime) {
        setGreen(2);
      } else if (currTime < flashCycleTime * 2) {
        setWhite(2);
      }if (currTime < flashCycleTime * 3) {
        setGreen(2);
      } else if (currTime < flashCycleTime * 4) {
        setWhite(2);
      }if (currTime < flashCycleTime * 5) {
        setGreen(2);
      } else if (currTime < flashCycleTime * 6) {
        setWhite(2);
      }if (currTime < flashCycleTime * 7) {
        setGreen(2);
      } else if (currTime < flashCycleTime * 8) {
        setWhite(2);
      } if (currTime < flashCycleTime * 9) {
        setGreen(2);
      } else if (currTime < flashCycleTime * 10) {
        setWhite(2);
      } else {
        setGreen(2);
      }
      
    } else {
      setRed(2);
    }

    if (Math.abs(6500 - SmartDashboard.getNumber("bottomVelo", currTime)) < 100) {
      ledBuffer.setRGB(ledBuffer.getLength(), 0, 0, 100);
    } else {
      ledBuffer.setRGB(ledBuffer.getLength(), 100, 100, 0);
    }

    if (Math.abs(6500 - SmartDashboard.getNumber("topVelo", currTime)) < 100) {
      ledBuffer.setRGB(ledBuffer.getLength() - 1, 0, 0, 100);
    } else {
      ledBuffer.setRGB(ledBuffer.getLength() - 1, 100, 100, 100);
    }
  }
}
