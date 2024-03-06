// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.ShooterPositions;

public class ShooterToSubwoofer extends Command {
  private PivotSubsystem m_pivot;
  private boolean isFinished = false;
  public ShooterToSubwoofer(FlywheelSubsystem shooter, PivotSubsystem pivot) {
    m_pivot = pivot;
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // GlobalVariables.shooterAutomatic = false;
    // m_pivot.position = ShooterPositions.SUBWOOFER;
    isFinished = true;
    isFinished();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}