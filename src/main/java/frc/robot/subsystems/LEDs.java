// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class LEDs extends SubsystemBase { 
  private AddressableLED ledstrip;
  private AddressableLEDBuffer ledBuffer;
  boolean ledInit = false;
  double initTime = 0;
  double currTime = 0;
  double flashCycleTime = .25;
  int shooterLeds = 36;
  String LEDstate = "off";
  boolean upToSpeed = false;
  boolean ShooterLedInit = false;
  double shooterInitTime = 0;
  double shooterCurrTime = 0;
  double shooterFlashCycleTime = 0.3;
  double shooterMaxError = 100;
  double endGameFlashCycleTime = .3;

  public LEDs() {
    ledstrip = new AddressableLED(Constants.LEDS.LEDPort);
    ledBuffer = new AddressableLEDBuffer(Constants.LEDS.numLEDs);
    ledstrip.setBitTiming(Constants.LEDS.highZero, Constants.LEDS.lowZero, Constants.LEDS.highOne, Constants.LEDS.lowOne);
    ledstrip.setLength(ledBuffer.getLength());
    ledstrip.setData(ledBuffer);
    ledstrip.start();
  }

  public void setRed(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "red";
  }

  public void setGreen(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "green";
  }

  public void flashGreen(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    ledstrip.setData(ledBuffer);
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 255, 255);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "green";
  }
  
  public void setBlue(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 255);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "blue";
  }

  public void setWhite(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 100, 100, 100);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "white";
  }

  public void setYellow(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 100, 100, 0);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "yellow";
  }

  public void setOff(int offFromMax){
    for (var i = offFromMax; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 255, 255);
    }
    ledstrip.setData(ledBuffer);
    LEDstate = "off";
  }
  
  // public void updateShooterLEDS() {
  //   for (var i = 0; i < shooterLeds; i++) {
  //       if (LEDstate == "off") {
  //         ledBuffer.setRGB(i, 0, 0, 0);
  //       } else if (LEDstate == "red") {
  //         ledBuffer.setRGB(i, 100, 0, 0);
  //       } else if (LEDstate == "green") {
  //         ledBuffer.setRGB(i, 0, 100, 0);
  //        }else if (LEDstate == "blue") {
  //         ledBuffer.setRGB(i, 0, 0, 0);
  //       } else if (LEDstate == "white") {
  //         ledBuffer.setRGB(i, 100,100, 100);
  //       }
  //     }
  // }

  @Override
  public void periodic() {
    if (GlobalVariables.Timing.teleopTimeElapsed < 115)  {
      if (!GlobalVariables.Conveyor.beamBroken) {
        if (!ledInit) {
          ledInit = true;
          initTime = Timer.getFPGATimestamp();
        }
        currTime = Timer.getFPGATimestamp() - initTime;

        if (currTime < flashCycleTime*50) {
          flashGreen(0);
        }
      } else {
        setOff(0);
        ledInit = false;
      }
    
      if (Math.abs(Constants.Flywheel.speakerVelo - GlobalVariables.Flywheel.topVelo) < shooterMaxError && Math.abs(Constants.Flywheel.speakerVelo - GlobalVariables.Flywheel.bottomVelo) < shooterMaxError) {
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
          // updateShooterLEDS();
          
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
        // updateShooterLEDS();
        ShooterLedInit = false;
      }
    } else if (GlobalVariables.Timing.teleopTimeElapsed < 130) {
      setYellow(0);
    } else {
      setRed(0);
    }
    ledstrip.setData(ledBuffer);
  }
}