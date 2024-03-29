// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberRight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RunServoRightCommand extends Command {
  
  public ClimberSubsystem m_climberRight;
  public double servoPosition;
  
  /** Creates a new RunServoCommand. */
  public RunServoRightCommand(ClimberSubsystem climber_R, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberRight = climber_R;
    servoPosition = pos;
    addRequirements(m_climberRight);
  }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberRight.setPositionRight(servoPosition);
  }

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
