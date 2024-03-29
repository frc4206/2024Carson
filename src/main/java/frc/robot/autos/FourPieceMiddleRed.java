// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Conveyor.ConveyorToDuty;
import frc.robot.commands.Intake.GoUntilNote;
import frc.robot.commands.Intake.IntakeToDuty;
import frc.robot.commands.Pivot.PivotToPosition;
import frc.robot.commands.Shooter.ShooterToDuty;
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
      new ShooterToDuty(flywheel, 1),
      new SequentialCommandGroup(
        new PivotToPosition(pivot, 4.4).withTimeout(0.02),
        new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-2.5, 5.50, 0, 1, true),
        new ConveyorToDuty(conveyor, 1).withTimeout(0.225),

        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-3.0, 5.50, 0, 0.5, false),
          new GoUntilNote(conveyor, intake).until(() -> conveyor.hasNote()),
          new PivotToPosition(pivot, 2.3).withTimeout(0.4)
        ),

        new ParallelCommandGroup(
          new ConveyorToDuty(conveyor, 0.2).withTimeout(0.55),
          new IntakeToDuty(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-3.95, 5.215, 360-355, 2, true)
        ),

        new ParallelCommandGroup(
          new IntakeToDuty(intake, -1).withTimeout(0.4), 
          new ConveyorToDuty(conveyor, 1).withTimeout(0.4)
        ),

        new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-5.50, 4.10, 0, 1.5, false),
        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-8.00, 4.10, 0, 2, false),
          new GoUntilNote(conveyor, intake).until(() -> conveyor.hasNote())
        ),

        new ParallelCommandGroup(
          new ConveyorToDuty(conveyor, 0.2).withTimeout(0.55),
          new IntakeToDuty(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-5.50, 4.10, 0, 2, false),
          new PivotToPosition(pivot, 3).withTimeout(0.5)
        ),

        new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-4.05, 5.05, 360-350, 2, true),
        new ConveyorToDuty(conveyor, 1).withTimeout(0.225),

        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-8.00, 2.675, 360-330, 3, false),
          new GoUntilNote(conveyor, intake).until(() -> conveyor.hasNote())
        ),

        new ParallelCommandGroup(
          new ConveyorToDuty(conveyor, 0.2).withTimeout(0.55),
          new IntakeToDuty(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-6.00, 4.10, 360-350, 1.5, false),
          new PivotToPosition(pivot, 3).withTimeout(0.5)
          ),
        new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-4.05, 5.05, 360-350, 2.5, false),
          
        new ConveyorToDuty(conveyor, 1).withTimeout(0.225)
      )
    );
  }
}
