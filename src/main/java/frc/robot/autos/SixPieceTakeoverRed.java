// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Conveyor.ConveyerToSpeedCommand;
import frc.robot.commands.Pivot.PivotCommand;
import frc.robot.commands.Swerve.PID_DistanceOdometry2;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixPieceTakeoverRed extends SequentialCommandGroup {
  public SixPieceTakeoverRed(ConveyorSubsystem conveyor, FlywheelSubsystem flywheel, IntakeSubsystem intake, PivotSubsystem pivot, SwerveSubsystem swerve) {
    addCommands(
      new PivotCommand(pivot, 3.5).withTimeout(0.02),
      new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-2.00, 4.10, 35, 1, false),
      new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.08)//,
      // new ParallelCommandGroup(
      //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-2.725, 4.10, 0, 1)//,
      //   // new GoUntilNote(conveyor, m_intakeSubsystem)
      // ),
      // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-2.00, 4.825, 19, 1),
      // new PivotCommand(pivot, 4).withTimeout(0.1),
      // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
      // new ParallelCommandGroup(
      //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-2.725, 5.55, 315, 1)//,
      //   // new GoUntilNote(conveyor, m_intakeSubsystem)
      // ),
      // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-2.00, 6.275, 341, 1),
      // new PivotCommand(pivot, 3.5).withTimeout(0.1),
      // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
      // new ParallelCommandGroup(
      //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-2.725, 7.00, 315, 1)//,
      //   // new GoUntilNote(conveyor, m_intakeSubsystem)
      // ),
      // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-3.80, 7.10, 337.5, 1.25),
      // new PivotCommand(pivot, 2.75).withTimeout(0.1),
      // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
      // new ParallelCommandGroup(
      //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-7.88, 7.44, 0, 5)//,
      //   // new GoUntilNote(conveyor, m_intakeSubsystem)
      // ),
      // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-5.10, 6.6, 349, 5),
      // new PivotCommand(pivot, 2.25).withTimeout(0.1),
      // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
      // new ParallelCommandGroup(
      //   new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-7.88, 5.78, 0, 5)//,
      //   //new GoUntilNote(conveyor, m_intakeSubsystem)
      // ),
      // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-5.10, 6.6, 349, 5),
      // new PivotCommand(pivot, 2.25).withTimeout(0.1),
      // new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.05),
      // new PID_DistanceOdometry2(swerve, true, true, Constants.Field.fieldWidth-6.88, 5.78, 0, 5)
    );
  }
}
