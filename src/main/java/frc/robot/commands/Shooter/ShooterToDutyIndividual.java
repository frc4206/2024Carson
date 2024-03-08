// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class ShooterToDutyIndividual extends Command {
  private FlywheelSubsystem m_flywheel;
  private double m_upperPercent;
  private double m_lowerPercent;
  public ShooterToDutyIndividual(FlywheelSubsystem flywheel, double upperPercent, double lowerPercent) {
    m_upperPercent = upperPercent;
    m_lowerPercent = lowerPercent;
    m_flywheel = flywheel;
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.percentShooter2(m_upperPercent, m_lowerPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.percentShooter2(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
