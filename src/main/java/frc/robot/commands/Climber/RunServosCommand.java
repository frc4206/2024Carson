// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RunServosCommand extends Command {
	private ClimberSubsystem m_climberSubsystem;
	private double leftServoPos;
	private double rightServoPos;

	public RunServosCommand(ClimberSubsystem climber, double leftServoPosition, double rightServoPosition) {
		leftServoPos = leftServoPosition;
		rightServoPos = rightServoPosition;
		m_climberSubsystem = climber;
		addRequirements(m_climberSubsystem);
	}

	/*public RunServosCommand(ClimberSubsystem climber, int servoChannel) {
		if(servoChannel == servoRight.getChannel()) {

		} else if(servoChannel == servoLeft.getChannel()) {
		}
		m_climberSubsystem = climber;
		addRequirements(m_climberSubsystem);
	}*/

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_climberSubsystem.setPositionBoth(leftServoPos, rightServoPos);
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
