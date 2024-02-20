// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ShooterToSpeaker extends Command {
  private FlywheelSubsystem m_shooter;
  private PivotSubsystem m_pivot;
  public ShooterToSpeaker(FlywheelSubsystem shooter, PivotSubsystem pivot) {
    m_shooter = shooter;
    m_pivot = pivot;
    addRequirements(m_shooter, m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GlobalVariables.shooterAutomatic = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setVelocity(6500);
    m_pivot.autoAdjust(GlobalVariables.distanceToSpeaker);
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
