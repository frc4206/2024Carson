// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public CANSparkFlex inMotor = new CANSparkFlex(14, MotorType.kBrushless);
  public double intakeSpeed;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(double inSpeed) {
    intakeSpeed = inSpeed;
  }

  // Change name at some point?
  public void intake() {
    inMotor.set(intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
