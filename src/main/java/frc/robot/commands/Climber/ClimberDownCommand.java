// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDownCommand extends Command {

	ClimberSubsystem left_climber;
	ClimberSubsystem right_climber;

	public ClimberDownCommand(ClimberSubsystem climber) {
		addRequirements(climber);
	}

	public ClimberDownCommand(ClimberSubsystem climber_left, ClimberSubsystem climber_right) {
		this.left_climber = climber_left;
		this.right_climber = climber_right;
		addRequirements(climber_left, climber_right);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		this.left_climber.climbDown(Climber.servoPosLeftDisEngage);
		this.right_climber.climbDown(Climber.servoPosRightDisEngage);
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
