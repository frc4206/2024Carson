// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Swerve.PID_DistanceOdometry2;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JustLeaveRed extends SequentialCommandGroup {
  public JustLeaveRed(SwerveSubsystem swerve, Pose2d initialPose) {
    addCommands(
      new WaitCommand(1),
      new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-initialPose.getX(), initialPose.getY(), 0, 0, true)
    );
  }
}
