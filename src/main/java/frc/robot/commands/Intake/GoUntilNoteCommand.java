// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GoUntilNoteCommand extends Command {
	private ConveyorSubsystem m_conveyor; 
	private IntakeSubsystem m_intake;
	public GoUntilNoteCommand(ConveyorSubsystem conveyor, IntakeSubsystem intake) {
		m_conveyor = conveyor;
		m_intake = intake;
		addRequirements(m_conveyor, m_intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		GlobalVariables.intakingPiece = true;
		GlobalVariables.pieceReady = false;
		GlobalVariables.ampReady = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(!m_conveyor.hasNote()) {
			m_intake.intakeGo(-0.75);
			m_conveyor.conveyorToDuty(0.4);
		} else {
			m_intake.intakeGo(0);
			m_conveyor.conveyorToDuty(0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_conveyor.resetConveyor();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
