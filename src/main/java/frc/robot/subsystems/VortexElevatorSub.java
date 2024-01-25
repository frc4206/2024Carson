// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VortexElevatorSub extends SubsystemBase {
  private CANSparkFlex elevator1 = new CANSparkFlex(21, MotorType.kBrushless);
  
  public VortexElevatorSub() {
    elevator1.restoreFactoryDefaults();
    elevator1.setInverted(false);
  }

  public void elevatorSTOP(){
    elevator1.set(0);
  }

  public void elevatorUP(){
    elevator1.set(0.8);
  }

  public void elevatorDOWN(){
    elevator1.set(-0.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
