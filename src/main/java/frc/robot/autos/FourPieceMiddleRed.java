// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
public class FourPieceMiddleRed extends ParallelCommandGroup {
  public FourPieceMiddleRed(ConveyorSubsystem conveyor, FlywheelSubsystem flywheel, IntakeSubsystem intake, PivotSubsystem pivot, SwerveSubsystem swerve) {
    addCommands(
      new PercentShooterCommand(flywheel, 1),
      new SequentialCommandGroup(
        new PivotCommand(pivot, 4.4).withTimeout(0.02),
        new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-2.5, 5.50, 0, 1, true),
        new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.225),

        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-3.0, 5.50, 0, 2, false),
          new GoUntilNoteCommand(conveyor, intake).until(() -> conveyor.hasNote()),
          new PivotCommand(pivot, 2.3).withTimeout(0.4)
        ),

        new ParallelCommandGroup(
          new ConveyerToSpeedCommand(conveyor, 0.2).withTimeout(0.55),
          new IntakeToSpeedCommand(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-3.95, 5.215, 5, 2, true)
        ),

        new ParallelCommandGroup(
          new IntakeToSpeedCommand(intake, -1).withTimeout(0.4), 
          new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.4)
        ),

        new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-5.50, 4.10, 0, 1.5, false),
        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-8.00, 4.10, 0, 3, false),
          new GoUntilNoteCommand(conveyor, intake).until(() -> conveyor.hasNote())
        ),

        new ParallelCommandGroup(
          new ConveyerToSpeedCommand(conveyor, 0.2).withTimeout(0.55),
          new IntakeToSpeedCommand(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-5.50, 4.10, 0, 2, false),
          new PivotCommand(pivot, 3).withTimeout(0.5)
        ),

        new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-4.05, 5.05, 10, 2, true),
        new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.225),

        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-7.93, 2.62, 30, 3, false),
          new GoUntilNoteCommand(conveyor, intake).until(() -> conveyor.hasNote())
        ),

        new ParallelCommandGroup(
          new ConveyerToSpeedCommand(conveyor, 0.2).withTimeout(0.55),
          new IntakeToSpeedCommand(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-4.05, 5.05, 10, 2, false),
          new PivotCommand(pivot, 3).withTimeout(0.5)
        ),

        new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.225)
      )
    );
  }
}
