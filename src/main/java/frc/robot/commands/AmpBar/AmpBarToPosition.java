// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpBar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpBarSubsystem;

public class AmpBarToPosition extends Command {
  private AmpBarSubsystem m_ampBar;
  private double m_desiredPosition;
  public AmpBarToPosition(AmpBarSubsystem ampBar, double desiredPosition) {
    m_ampBar = ampBar;
    m_desiredPosition = desiredPosition;
    addRequirements(m_ampBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ampBar.setPosition(m_desiredPosition);
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
