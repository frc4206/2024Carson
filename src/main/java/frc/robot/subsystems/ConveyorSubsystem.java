// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ConveryorSubsystem. */
  private CANSparkFlex conveyorMotor = new CANSparkFlex(Constants.Conveyor.conveyorMotorID, MotorType.kBrushless);

  public ConveyorSubsystem() {}

  public void conveyorTurn(double conveyorSpeed) {
    conveyorMotor.set(conveyorSpeed);
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
