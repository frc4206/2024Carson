// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class DistanceVelocityCommand extends Command {
  /** Creates a new DistanceVelocitcyCommand. */
  FlywheelSubsystem m_flywheelSubsystem;
  public DistanceVelocityCommand(FlywheelSubsystem flywheelSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flywheelSubsystem = flywheelSubsystem;
    addRequirements(flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheelSubsystem.setVelocity(SmartDashboard.getNumber("desired velo", 0));
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
