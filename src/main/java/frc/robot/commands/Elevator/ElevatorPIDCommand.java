// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.VortexElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCommand extends Command {
	private ElevatorSubsystem m_VortexElevatorSubsystem;
	private double m_setpoint;
	
	public ElevatorPIDCommand(ElevatorSubsystem vortexElevator, double setpoint) {
		m_VortexElevatorSubsystem = vortexElevator;
		m_setpoint = setpoint;
		addRequirements(vortexElevator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_VortexElevatorSubsystem.GoToSetpoint(m_setpoint);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
  	}
}
