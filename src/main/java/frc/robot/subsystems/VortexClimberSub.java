// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VortexClimberSub extends SubsystemBase {
  private CANSparkFlex climber1 = new CANSparkFlex(20, MotorType.kBrushless);

  public VortexClimberSub() {
    climber1.restoreFactoryDefaults();
    climber1.setInverted(false);
  }

  public void climbSTOP(){
    climber1.set(0);
  }

  public void climbUP(){
    climber1.set(0.8);
  }

  public void climbDOWN(){
    climber1.set(-0.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
