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
public class SixPieceTakeoverRed extends ParallelCommandGroup {
  public SixPieceTakeoverRed(ConveyorSubsystem conveyor, FlywheelSubsystem flywheel, IntakeSubsystem intake, PivotSubsystem pivot, SwerveSubsystem swerve) {
    addCommands(
      new ShooterToDuty(flywheel, 1),
      new SequentialCommandGroup(
        new PivotToPosition(pivot, 4.65).withTimeout(0.02),
        new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-1.9, 4.00, 360-325, 1, false),
        new ConveyorToDuty(conveyor, 1).withTimeout(0.225),
        new ParallelCommandGroup(
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-2.725, 4.00, 0, 1, false),
          new GoUntilNote(conveyor, intake).until(() -> conveyor.hasNote())
          ),
        new ParallelCommandGroup(
          new ConveyorToDuty(conveyor, 0.2).withTimeout(0.55),
          new IntakeToDuty(intake, -0.1).withTimeout(0.55),
          new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-2.35, 4.805, 360-355, 1, false),
          new PivotToPosition(pivot, 4.65).withTimeout(0.1)
        ),
        new ConveyorToDuty(conveyor, 1).withTimeout(0.225)
        // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
        // new ParallelCommandGroup(
          //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-2.675, 5.55, 360-45, 1)//,
          //   // new GoUntilNote(conveyor, m_intakeSubsystem)
          // ),
          // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-2.00, 6.275, 360-19, 1),
          // new PivotCommand(pivot, 3.5).withTimeout(0.1),
          // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
          // new ParallelCommandGroup(
            //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-2.725, 7.00, 360-45, 1)//,
            //   // new GoUntilNote(conveyor, m_intakeSubsystem)
            // ),
            // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-3.80, 7.10, 360-22.5, 1.25),
            // new PivotCommand(pivot, 2.75).withTimeout(0.1),
            // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
            // new ParallelCommandGroup(
              //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-7.88, 7.44, 0, 5)//,
              //   // new GoUntilNote(conveyor, m_intakeSubsystem)
              // ),
              // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-5.10, 6.6, 360-11, 5),
              // new PivotCommand(pivot, 2.25).withTimeout(0.1),
              // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
              // new ParallelCommandGroup(
                //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-7.88, 5.78, 0, 5)//,
                //   //new GoUntilNote(conveyor, m_intakeSubsystem)
                // ),
                // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-5.10, 6.6, 360-11, 5),
                // new PivotCommand(pivot, 2.25).withTimeout(0.1),
                // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
                // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldLength-6.88, 5.78, 0, 5)
      )    
    );
  }
}