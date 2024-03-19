// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMBOS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Pivot.GetPivotToPosition;
import frc.robot.commands.Shooter.GetShooterToVelocity;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootAtSpeakerCommand extends SequentialCommandGroup {
  public AutoShootAtSpeakerCommand(SwerveSubsystem swerve, FlywheelSubsystem flywheelSubsystem, PivotSubsystem pivotsubsystem) {
    addCommands(
      new GetShooterToVelocity(flywheelSubsystem, 6500), 
      new GetPivotToPosition(pivotsubsystem, Constants.Pivot.subwooferPosition)
    );
  }
}
