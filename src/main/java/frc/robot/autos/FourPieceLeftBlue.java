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
public class FourPieceLeftBlue extends ParallelCommandGroup {
  public FourPieceLeftBlue(ConveyorSubsystem conveyor, FlywheelSubsystem flywheel, IntakeSubsystem intake, PivotSubsystem pivot, SwerveSubsystem swerve) {
    addCommands(
      // new PercentShooterCommand(flywheel, 1),
      new SequentialCommandGroup(
        // new PivotCommand(pivot, 4.4).withTimeout(0.02),
        new PID_DistanceOdometry2(swerve, true, true, 2.25, 7, 34.3, 1, false),
        // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.225),

        new ParallelCommandGroup(
          // new PivotCommand(pivot, 4.4).withTimeout(0.1),
          new PID_DistanceOdometry2(swerve, true, true, 2.725, 7, 28.8, 1, true)//,
          // new GoUntilNoteCommand(conveyor, intake)
        ),

        new ParallelCommandGroup(
          // new ConveyerToSpeedCommand(conveyor, 0.2).withTimeout(0.55),
          // new IntakeToSpeedCommand(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, 4.10, 7.05, 20.7, 2, true)
        ),

        new ParallelCommandGroup(
          // new IntakeToSpeedCommand(intake, -1).withTimeout(0.4), 
          // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.4)
        ),

        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, 7.88, 7.45, 0, 3, false)//,
          // new GoUntilNoteCommand(conveyor, intake).until(() -> conveyor.hasNote())
        ),

        new ParallelCommandGroup(
          // new ConveyerToSpeedCommand(conveyor, 0.2).withTimeout(0.55),
          // new IntakeToSpeedCommand(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, 3.25, 6.875, 18, 2, true)//,
          // new PivotCommand(pivot, 3).withTimeout(0.5)
        ),

        new ParallelCommandGroup(
          // new IntakeToSpeedCommand(intake, -1).withTimeout(0.4), 
          // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.4)
        ),

        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, 7.83, 5.96, 0, 3, false)//,
          // new GoUntilNoteCommand(conveyor, intake).until(() -> conveyor.hasNote())
        ),

        new ParallelCommandGroup(
          // new ConveyerToSpeedCommand(conveyor, 0.2).withTimeout(0.55),
          // new IntakeToSpeedCommand(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, 3.25, 6.875, 0, 2, true)//,
          // new PivotCommand(pivot, 3).withTimeout(0.5)
          ), 
          
        new PID_DistanceOdometry2(swerve, true, true, 3.9, 5.95, 13.3, 2, true),

        new ParallelCommandGroup(
          // new IntakeToSpeedCommand(intake, -1).withTimeout(0.4), 
          // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.4)
        )
      )
    );
  }
}
