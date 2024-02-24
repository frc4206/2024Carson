// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Conveyor.ConveyerToSpeedCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoUntilNote extends SequentialCommandGroup {
	public GoUntilNote(ConveyorSubsystem conveyor, IntakeSubsystem intake) {
		addCommands(
		new GoUntilNoteCommand(conveyor, intake).until(() -> conveyor.hasNote()),
		new ParallelCommandGroup(
			new ConveyerToSpeedCommand(conveyor, 0.2),
			new IntakeToSpeedCommand(intake, -0.1)
		).withTimeout(0.55)
		);
	}
}
