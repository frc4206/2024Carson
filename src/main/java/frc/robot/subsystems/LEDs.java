// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;

public class LEDs extends SubsystemBase { 
  private AddressableLED ledstrip;
  private AddressableLEDBuffer ledBuffer;
  boolean ledInit = false;
  double initTime = 0;
  double currTime = 0;
  double flashCycleTime = .1;
  int shooterLeds = 24;
  String LEDstate = "off";
  boolean upToSpeed = false;
  boolean ShooterLedInit = false;
  double shooterInitTime = 0;
  double shooterCurrTime = 0;
  double shooterFlashCycleTime = 0.3;
  double shooterMaxError = 100;
  double endGameFlashCycleTime = .3;

  public LEDs() {
    ledstrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(48);
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
    LEDstate = "red";
  }
  public void setRed(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "red";
  }

  public void setGreen(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "green";
  }
  public void setGreen(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "green";
  }
  public void flashGreen(int offFromMax){
    for (var i = 0; i < ledBuffer.getLength() - offFromMax; i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    ledstrip.setData(ledBuffer);
    for (var i = 0; i < ledBuffer.getLength() - offFromMax; i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    ledstrip.setData(ledBuffer);
  }
  

  public void setBlue(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 255);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "white";
  }

  public void setWhite(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 100, 100, 100);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "white";
  }
  public void setWhite(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 100, 100, 100);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "white";
  }
  public void setYellow(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 100, 100, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "yellow";
  }

  public void setOff(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 100, 100, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "off";
  }
  public void setOff(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 100, 100, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "off";
  }

  public void updateShooterLEDS() {
    for (var i = 0; i < shooterLeds; i++) {
        if (LEDstate == "off") {
          ledBuffer.setRGB(i, 0, 0, 0);
        } else if (LEDstate == "red") {
          ledBuffer.setRGB(i, 100, 0, 0);
        } else if (LEDstate == "green") {
          ledBuffer.setRGB(i, 0, 100, 0);
         }else if (LEDstate == "blue") {
          ledBuffer.setRGB(i, 0, 0, 0);
        } else if (LEDstate == "white") {
          ledBuffer.setRGB(i, 100,100, 100);
        }
      }
  }

  @Override
  public void periodic() {
    if (GlobalVariables.teleopTimeElapsed < 115)  {
       if (!SmartDashboard.getBoolean("beam broken", false)) {
        if (!ledInit) {
          ledInit = true;
          initTime = Timer.getFPGATimestamp();
        }
        currTime = Timer.getFPGATimestamp() - initTime;

        if (currTime < flashCycleTime) {
          setGreen(shooterLeds);
        } else if (currTime < flashCycleTime * 2) {
          setWhite(shooterLeds);
        }if (currTime < flashCycleTime * 3) {
          setGreen(shooterLeds);
        } else if (currTime < flashCycleTime * 4) {
          setWhite(shooterLeds);
        }if (currTime < flashCycleTime * 5) {
          setGreen(shooterLeds);
        } else if (currTime < flashCycleTime * 6) {
          setWhite(shooterLeds);
        }if (currTime < flashCycleTime * 7) {
          setGreen(shooterLeds);
        } else if (currTime < flashCycleTime * 8) {
          setWhite(shooterLeds);
        } if (currTime < flashCycleTime * 9) {
          setGreen(shooterLeds);
        } else if (currTime < flashCycleTime * 10) {
          setWhite(shooterLeds);
        } else {
          setGreen(shooterLeds);
        }

      } else {
        setWhite(shooterLeds);
        ledInit = false;
      }
    
      if (Math.abs(6500 - SmartDashboard.getNumber("bottomVelo", currTime)) < shooterMaxError && Math.abs(6500 - SmartDashboard.getNumber("topVelo", currTime)) < shooterMaxError) {
        upToSpeed = true;
      } else {
        upToSpeed = false;
      }
      if (upToSpeed) {
        if (!ShooterLedInit) {
          ShooterLedInit = true;
          shooterInitTime = Timer.getFPGATimestamp();
        }
        shooterCurrTime = Timer.getFPGATimestamp() - shooterInitTime;
        if (shooterCurrTime < shooterFlashCycleTime) {
          for (var i = 0; i < shooterLeds; i++) {
            ledBuffer.setRGB(i, 0, 0, 100);
          }
        } else if (shooterCurrTime < shooterFlashCycleTime * 2) {
          updateShooterLEDS();
          
        } else if (shooterCurrTime < shooterFlashCycleTime *3) {
          for (var i = 0; i < shooterLeds; i++) {
            ledBuffer.setRGB(i, 0, 0, 100);
          }
          ShooterLedInit = false;
        }
        // for (var i = 0; i < shooterLeds; i++) {
        //     ledBuffer.setRGB(i, 0, 0, 100);
        //   }
        ledstrip.setData(ledBuffer);
          
      } else {
        updateShooterLEDS();
        ShooterLedInit = false;
      }
    } else if (GlobalVariables.teleopTimeElapsed < 130) {
      setYellow();
    } else {
      setRed();
    }
    ledstrip.setData(ledBuffer);
    // SmartDashboard.putString("LEDstate", LEDstate);
    // SmartDashboard.putBoolean("up to speed", upToSpeed);
  }
}