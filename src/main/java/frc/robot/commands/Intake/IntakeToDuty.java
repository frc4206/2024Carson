// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToDuty extends Command {
	private IntakeSubsystem m_intake;
	private double m_intakeSpeed;
	public IntakeToDuty(IntakeSubsystem intake, double intakeSpeed) {
		m_intake = intake; 
		addRequirements(m_intake);
		m_intakeSpeed = intakeSpeed;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_intake.intakeGo(m_intakeSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_intake.intakeGo(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
