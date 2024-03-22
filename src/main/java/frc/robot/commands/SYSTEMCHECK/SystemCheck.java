// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SYSTEMCHECK;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Conveyor.ConveyorToDuty;
import frc.robot.commands.Intake.IntakeToDuty;
import frc.robot.commands.Intake.SetupNote;
import frc.robot.commands.Pivot.PivotToPosition;
import frc.robot.commands.Shooter.ShooterToVelocity;
import frc.robot.commands.Shooter.ShooterToVelocityIndividual;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemCheck extends SequentialCommandGroup {
  public SystemCheck(ClimberSubsystem climber, ConveyorSubsystem conveyor, FlywheelSubsystem flywheel, IntakeSubsystem intake, PivotSubsystem pivot, SwerveSubsystem swerve, XboxController controller) {
    addCommands(
      new IntakeToDuty(intake, -1).withTimeout(0.25),
      new IntakeToDuty(intake, 1).withTimeout(0.25),
      new ConveyorToDuty(conveyor, 1).withTimeout(0.25),
      new ConveyorToDuty(conveyor, -1).withTimeout(0.25),
      new SetupNote(conveyor, intake),
      new ShooterToVelocity(flywheel, 6500).until(() -> FlywheelSubsystem.shooterAtVelocity(6500)),
      new ShooterToVelocityIndividual(flywheel, 3100, 1800).until(() -> FlywheelSubsystem.shooterAtVelocities(3100, 1800)),
      new PivotToPosition(pivot, Constants.Pivot.stagePosition).withTimeout(0.5),
      new PivotToPosition(pivot, Constants.Pivot.podiumPosition).withTimeout(0.25),
      new PivotToPosition(pivot, Constants.Pivot.underPosition).withTimeout(0.25),
      new PivotToPosition(pivot, Constants.Pivot.closePosition).withTimeout(0.25),
      new TeleopSwerve(swerve, controller, 1, 0, 4, true, true).withTimeout(5)
    );
  }
}
