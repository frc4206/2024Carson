// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class GetPivotToPosition extends Command {
	private PivotSubsystem m_pivot;
	private double m_position;
	private boolean isFinished = false;
	public GetPivotToPosition(PivotSubsystem pivot, double position) {
		m_pivot = pivot;
		addRequirements(m_pivot);
		m_position = position;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_pivot.pivotToPosition(m_position);
		if (m_pivot.pivotAtPosition(m_position)) {
			isFinished = true;
			isFinished();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return isFinished;
	}
}
