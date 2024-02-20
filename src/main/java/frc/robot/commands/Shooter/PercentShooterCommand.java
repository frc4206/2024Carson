// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class PercentShooterCommand extends Command {
  /** Creates a new ShooterIntakeCommand. */
  FlywheelSubsystem m_flywheelSubsystem;
  double percent;
  public PercentShooterCommand(FlywheelSubsystem flywheelSubsystem, double per) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flywheelSubsystem = flywheelSubsystem;
    percent = per;
    addRequirements(flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheelSubsystem.percentShooter(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_flywheelSubsystem.percentShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
