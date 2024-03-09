// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberSetSpeedCommand extends Command {

	private ClimberSubsystem climberLeft;
	private ClimberSubsystem climberRight;
	double leftMotorSpeed;
	double rightMotorSpeed;
	public XboxController controller;

	public enum MotorDirections {
		UP,
		DOWN,
		STOP
	}
	public MotorDirections direction;

	/** Creates a new ClimberSetSpeedCommand. */
	public ClimberSetSpeedCommand(ClimberSubsystem climber_L, ClimberSubsystem climber_R, MotorDirections motorRunDirection, int controllerPort) {
		climberLeft = climber_L;
		climberRight = climber_R;
		direction = motorRunDirection;
		controller = new XboxController(controllerPort);
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(climberLeft, climberRight);
	}

	public ClimberSetSpeedCommand(ClimberSubsystem climber, MotorDirections motorRunDirection, int controllerPort) {
		climberLeft = climber;
		climberRight = climber;
		direction = motorRunDirection;
		controller = new XboxController(controllerPort);
		addRequirements(climberLeft);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(direction == MotorDirections.UP) {
			climberLeft.climbUp(Climber.servoPosLeftDisEngage, controller.getLeftTriggerAxis());
			climberRight.climbUp(Climber.servoPosRightDisEngage, controller.getRightTriggerAxis());
		} else if(direction == MotorDirections.DOWN) {
			climberLeft.climbDown(Climber.servoPosLeftDisEngage);
			climberRight.climbDown(Climber.servoPosLeftDisEngage);
		} else {
			climberLeft.climbStop(Climber.servoPosLeftEngage);
			climberRight.climbStop(Climber.servoPosRightEngage);
		}

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
