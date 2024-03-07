// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climber.ClimberLeft.ClimbUpLeftCommand;
import frc.robot.commands.Climber.ClimberRight.ClimbUpRightCommand;
import frc.robot.subsystems.ClimbLeftSubsystem;
import frc.robot.subsystems.ClimbRightSubystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbUpCommand extends ParallelCommandGroup {
  public ClimbUpCommand(ClimbLeftSubsystem climberLeft, ClimbRightSubystem climberRight) {
    addCommands(
      new ClimbUpLeftCommand(climberLeft),
      new ClimbUpRightCommand(climberRight)
    );
  }
}