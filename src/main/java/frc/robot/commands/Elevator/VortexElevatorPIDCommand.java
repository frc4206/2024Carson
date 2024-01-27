// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VortexElevatorSub;

public class VortexElevatorPIDCommand extends Command {
  /** Creates a new VortexElevatorPIDCommand. */
  VortexElevatorSub m_VortexElevatorSub;

  private SparkPIDController elvatorLeadPid;

  public VortexElevatorPIDCommand(VortexElevatorSub VortexElevatorSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_VortexElevatorSub = VortexElevatorSub;
    addRequirements(VortexElevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_VortexElevatorSub.GoToSetpoint(100);
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
