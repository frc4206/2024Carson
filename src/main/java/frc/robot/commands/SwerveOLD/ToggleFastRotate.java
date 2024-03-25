// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveOLD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;

public class ToggleFastRotate extends Command {
  private boolean isFinished = false;
  public ToggleFastRotate() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (GlobalVariables.Swerve.rotationMultiplier == 1){
      GlobalVariables.Swerve.rotationMultiplier = 2.5;
    } else if (GlobalVariables.Swerve.rotationMultiplier == 2.5){
      GlobalVariables.Swerve.rotationMultiplier = 1;
    }
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