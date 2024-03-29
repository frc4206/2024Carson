// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDown extends Command {
	private ElevatorSubsystem m_vortexElevatorSubsystem;

	public ElevatorDown(ElevatorSubsystem vortexElevator) {
		m_vortexElevatorSubsystem = vortexElevator;
		addRequirements(vortexElevator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_vortexElevatorSubsystem.elevatorToPercent(-0.55);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_vortexElevatorSubsystem.elevatorToPercent(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
