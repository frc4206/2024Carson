// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SYSTEMCHECK;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Conveyor.ConveyorToDuty;
import frc.robot.commands.Elevator.ElevatorToPosition;
import frc.robot.commands.Intake.IntakeToDuty;
import frc.robot.commands.Pivot.PivotToPosition;
import frc.robot.commands.Shooter.ShooterToDuty;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemCheck extends SequentialCommandGroup {
  public SystemCheck(ClimberSubsystem leftClimber, ClimberSubsystem rightClimber, ConveyorSubsystem conveyor, ElevatorSubsystem elevator, FlywheelSubsystem flywheel, IntakeSubsystem intake, PivotSubsystem pivot, SwerveSubsystem swerve, XboxController controller) {
    addCommands(
      new IntakeToDuty(intake, -1).withTimeout(0.25),
      new IntakeToDuty(intake, 1).withTimeout(0.25),
      new ConveyorToDuty(conveyor, 1).withTimeout(0.25),
      new ConveyorToDuty(conveyor, -1).withTimeout(0.25),
      new ShooterToDuty(flywheel, 1).until(() -> flywheel.shooterAtVelocity(6000)),
      new PivotToPosition(pivot, Constants.Pivot.stagePosition).withTimeout(0.5),
      new PivotToPosition(pivot, Constants.Pivot.podiumPosition).withTimeout(0.25),
      new PivotToPosition(pivot, Constants.Pivot.underPosition).withTimeout(0.125),
      new PivotToPosition(pivot, Constants.Pivot.closePosition).withTimeout(0.125),
      new ElevatorToPosition(elevator, Constants.Elevator.elevatorTrapPosition).withTimeout(2),
      new ElevatorToPosition(elevator, 1).withTimeout(2),
      new TeleopSwerve(swerve, controller, 1, 0, 4, true, true).withTimeout(5)
    );
  }
}
