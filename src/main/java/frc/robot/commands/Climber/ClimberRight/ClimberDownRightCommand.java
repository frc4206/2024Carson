// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberRight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDownRightCommand extends Command {
	private ClimberSubsystem m_climberRight;
	double start = 0;
	double currtime = 0;

	public ClimberDownRightCommand(ClimberSubsystem climber_R) {
		m_climberRight = climber_R;
		addRequirements(m_climberRight);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		start = Timer.getFPGATimestamp();
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_climberRight.setPositionRight(Constants.Climber.servoPosRightDisEngage);
		currtime = Timer.getFPGATimestamp() - start;
		if (currtime > 0.2) {
		m_climberRight.climbDOWNRight();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_climberRight.climbSTOP();
		m_climberRight.setPositionRight(Constants.Climber.servoPosRightEngage);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
