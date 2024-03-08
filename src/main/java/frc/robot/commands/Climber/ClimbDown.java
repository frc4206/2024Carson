// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climber.ClimberLeft.ClimbDownLeft;
import frc.robot.commands.Climber.ClimberRight.ClimbDownRight;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbDown extends ParallelCommandGroup {
  public ClimbDown(ClimberSubsystem climberLeft, ClimberSubsystem climberRight) {
    addCommands(
      new ClimbDownLeft(climberLeft),
      new ClimbDownRight(climberRight)
    );
  }
}
