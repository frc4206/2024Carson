// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RunServoLeftCommand extends Command {
	public ClimberSubsystem m_climberSubsystem;
	public double servoPosition;
	
	/** Creates a new RunServoCommand. */
	public RunServoLeftCommand(ClimberSubsystem climber_L, double pos) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_climberSubsystem = climber_L;
		servoPosition = pos;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_climberSubsystem.setServoPosition(servoPosition);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// m_intakeMotor.intakeGo(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
