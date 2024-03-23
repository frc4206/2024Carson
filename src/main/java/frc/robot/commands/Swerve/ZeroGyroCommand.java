// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class ZeroGyroCommand extends Command {
	private Swerve swerveSubsystem;
	private boolean isfinished = false;
	
	public ZeroGyroCommand(Swerve m_swerveSubsystem) {
		swerveSubsystem = m_swerveSubsystem;
		addRequirements(swerveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerveSubsystem.zeroGyro();
		isfinished = true;
		isFinished();
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
		return isfinished;
	}
}
