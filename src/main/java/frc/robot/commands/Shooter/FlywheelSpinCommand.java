// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelSpinCommand extends Command {
  public FlywheelSubsystem flywheelSubsystem;
  private double flySpeed;
  public FlywheelSpinCommand(FlywheelSubsystem flywheelSubsystem, double FLYspeed) {
    this.flywheelSubsystem = flywheelSubsystem;
    flySpeed = FLYspeed;
    addRequirements(flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheelSubsystem.setVelocity(flySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
