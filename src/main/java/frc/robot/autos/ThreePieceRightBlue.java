// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Conveyor.ConveyerToSpeedCommand;
import frc.robot.commands.Intake.GoUntilNoteCommand;
import frc.robot.commands.Intake.IntakeToSpeedCommand;
import frc.robot.commands.Pivot.PivotCommand;
import frc.robot.commands.Shooter.PercentShooterCommand;
import frc.robot.commands.Swerve.PID_DistanceOdometry2;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceRightBlue extends ParallelCommandGroup {
  public ThreePieceRightBlue(ConveyorSubsystem conveyor, FlywheelSubsystem flywheel, IntakeSubsystem intake, PivotSubsystem pivot, SwerveSubsystem swerve) {
    addCommands(
      // new PercentShooterCommand(flywheel, 1),
      new SequentialCommandGroup(
        // new PivotCommand(pivot, 4.65).withTimeout(0.02),
        new PID_DistanceOdometry2(swerve, true, true, 1.9, 4.00, 325, 1, false),
        // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.225),
        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, 2.725, 4.00, 0, 1.5, false)//,
          // new GoUntilNoteCommand(conveyor, intake).until(() -> conveyor.hasNote())
          ),
        new ParallelCommandGroup(
          // new ConveyerToSpeedCommand(conveyor, 0.2).withTimeout(0.55),
          // new IntakeToSpeedCommand(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, 2.00, 4.10, 325, 1, true)//,
          // new PivotCommand(pivot, 4.65).withTimeout(0.1)
        ),
        // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.225),
        new PID_DistanceOdometry2(swerve, true, true, 5.00, 1.10, 0, 3, false),
        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, 7.88, 0.77, 0, 2.5, false)//,
          // new GoUntilNoteCommand(conveyor, intake).until(() -> conveyor.hasNote())
        ),
        new ParallelCommandGroup(
          // new ConveyerToSpeedCommand(conveyor, 0.2).withTimeout(0.55),
          // new IntakeToSpeedCommand(intake, -0.1).withTimeout(0.55),
          // new PivotCommand(pivot, 3.9).withTimeout(0.25),
          new PID_DistanceOdometry2(swerve, true, true, 4.00, 1.9, 0, 3, false)
        ),
        new PID_DistanceOdometry2(swerve, true, true, 2.00, 3.70, 318, 2, true)//,
        // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.225)
      )
    );
  }
}
