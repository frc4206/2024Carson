// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class GetShooterToVelocity extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;
	private double m_flySpeed;
	private double shooterMaxError = 100;
	private boolean isFinished = false;
	public GetShooterToVelocity(FlywheelSubsystem flywheelSubsystem, double flySpeed) {
		m_flywheelSubsystem = flywheelSubsystem;
		m_flySpeed = flySpeed;
		addRequirements(m_flywheelSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_flywheelSubsystem.setVelocity(m_flySpeed);
		if (Math.abs(6500 - SmartDashboard.getNumber("bottomVelo", 0)) < shooterMaxError && Math.abs(6500 - SmartDashboard.getNumber("topVelo", 0)) < shooterMaxError) {
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
