// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class ShooterToVelocity extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;
	private double m_flySpeed;
	public ShooterToVelocity(FlywheelSubsystem flywheelSubsystem, double flySpeed) {
		m_flywheelSubsystem = flywheelSubsystem;
		m_flySpeed = flySpeed;

		addRequirements(m_flywheelSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_flywheelSubsystem.setVelocity(m_flySpeed);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
