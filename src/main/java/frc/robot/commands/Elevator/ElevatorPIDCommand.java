// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCommand extends Command {
  
  ElevatorSubsystem m_VortexElevatorSubsystem;

  private SparkPIDController elvatorLeadPid;

  /** Oh boy, it's PID time! */
  public ElevatorPIDCommand(ElevatorSubsystem vortexElevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_VortexElevatorSubsystem = vortexElevator;
    addRequirements(vortexElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_VortexElevatorSubsystem.GoToSetpoint(-100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}