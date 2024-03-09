// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberRight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUpRightCommand extends Command {
	private ClimberSubsystem m_climberRight;
	private double startTime = 0;
	private double currTime = 0;
	private XboxController controller;
	
	public ClimbUpRightCommand(ClimberSubsystem climberRight, int controllerPort) {
		m_climberRight = climberRight;
		controller = new XboxController(controllerPort);
		addRequirements(m_climberRight);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_climberRight.setServoPosition(Constants.Climber.servoPosRightDisEngage);
		currTime = Timer.getFPGATimestamp() - startTime;
		if (currTime > 0.2) {
			m_climberRight.climbUp();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		//m_climberRight.setServoPosition(Constants.Climber.servoPosRightEngage);
		m_climberRight.climbStop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}