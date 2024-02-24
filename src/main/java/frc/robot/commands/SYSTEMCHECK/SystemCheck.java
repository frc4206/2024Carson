// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SYSTEMCHECK;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Climber.ClimberPIDCommand;
import frc.robot.commands.Conveyor.ConveyerToSpeedCommand;
import frc.robot.commands.Elevator.ElevatorPIDCommand;
import frc.robot.commands.Intake.IntakeToSpeedCommand;
import frc.robot.commands.Pivot.PivotCommand;
import frc.robot.commands.Shooter.PercentShooterCommand;
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
	public SystemCheck(ClimberSubsystem climber, ConveyorSubsystem conveyor, ElevatorSubsystem elevator, FlywheelSubsystem flywheel, IntakeSubsystem intake, PivotSubsystem pivot, SwerveSubsystem swerve, XboxController controller) {
		addCommands(
			new IntakeToSpeedCommand(intake, -1).withTimeout(0.25),
			new IntakeToSpeedCommand(intake, 1).withTimeout(0.25),
			new ConveyerToSpeedCommand(conveyor, 1).withTimeout(0.25),
			new ConveyerToSpeedCommand(conveyor, -1).withTimeout(0.25),
			new PercentShooterCommand(flywheel, 1).until(() -> flywheel.shooterAtVelocity(6000)),
			new PivotCommand(pivot, 4).withTimeout(1),
			new PivotCommand(pivot, 0.5).withTimeout(1),
			new ElevatorPIDCommand(elevator, Constants.Elevator.elevHighPosition).withTimeout(2),
			new ElevatorPIDCommand(elevator, 1).withTimeout(2),
			new ClimberPIDCommand(climber, 100).withTimeout(2),
			new ClimberPIDCommand(climber, 1).withTimeout(2),
			new TeleopSwerve(swerve, controller, 1, 0, 4, true, true).withTimeout(5)
		);
	}
}
