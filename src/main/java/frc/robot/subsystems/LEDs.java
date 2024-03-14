// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private AddressableLED ledstrip;
  private AddressableLEDBuffer ledBuffer;

  public LEDs() {
    ledstrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(60);
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

  public void setGreen(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
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

  @Override
  public void periodic() {

  }
}
