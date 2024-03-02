// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbLeftSubsystem;

public class ClimbLeftToPosition extends Command {
  private ClimbLeftSubsystem m_climberLeft;
  private double m_desiredPosition;
  public ClimbLeftToPosition(ClimbLeftSubsystem climberLeft, double desiredPosition) {
    m_climberLeft = climberLeft;
    m_desiredPosition = desiredPosition;
    addRequirements(m_climberLeft);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberLeft.climbToPosition(m_desiredPosition);
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
