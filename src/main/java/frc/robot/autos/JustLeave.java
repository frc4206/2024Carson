// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JustLeave extends SequentialCommandGroup {
  public JustLeave(SwerveSubsystem swerve) {
    addCommands(
      new WaitCommand(1),
      new RunCommand(() -> swerve.drive(new Translation2d(1.25, 0), 0, true, true), swerve).withTimeout(2.5)
    );
  }
}
