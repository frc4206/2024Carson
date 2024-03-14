// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class ShooterToVelocityIndividual extends Command {
  private FlywheelSubsystem m_flywheel;
  private double m_topDesiredVelo;
  private double m_bottomDesiredVelo;
  public ShooterToVelocityIndividual(FlywheelSubsystem flywheel, double topDesiredVelo, double bottomDesiredVelo) {
    m_flywheel = flywheel;
    m_topDesiredVelo = topDesiredVelo;
    m_bottomDesiredVelo = bottomDesiredVelo;
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setVelocity(m_topDesiredVelo, m_bottomDesiredVelo);
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
