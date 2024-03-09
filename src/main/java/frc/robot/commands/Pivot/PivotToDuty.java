// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToDuty extends Command {
	private PivotSubsystem pivot;
	private double m_percent;
	
	public PivotToDuty(PivotSubsystem m_pivot, double percent) {
		pivot = m_pivot;
		m_percent = percent;
		addRequirements(pivot);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		pivot.runPivot(m_percent);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		pivot.runPivot(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
