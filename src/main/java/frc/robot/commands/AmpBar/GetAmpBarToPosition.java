// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpBar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpBarSubsystem;

public class GetAmpBarToPosition extends Command {
  private AmpBarSubsystem m_ampBar;
  private double m_desiredPosition;
  private boolean isFinished = false;
  public GetAmpBarToPosition(AmpBarSubsystem ampBar, double desiredPosition) {
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
    m_ampBar.ampBarToPosition(m_desiredPosition);
    if (m_ampBar.ampBarWithinRange(m_desiredPosition)){
      isFinished = true;
      isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
