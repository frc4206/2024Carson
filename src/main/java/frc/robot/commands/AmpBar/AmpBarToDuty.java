// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpBar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpBarSubsystem;

public class AmpBarToDuty extends Command {
  private AmpBarSubsystem m_ampBar;
  private double m_desiredDuty;
  public AmpBarToDuty(AmpBarSubsystem ampBar, double desiredDuty) {
    m_ampBar = ampBar;
    m_desiredDuty = desiredDuty;
    addRequirements(m_ampBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ampBar.runAmpBar(m_desiredDuty);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ampBar.runAmpBar(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
