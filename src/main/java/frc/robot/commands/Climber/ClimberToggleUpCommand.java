// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberLeftSubsystem;
import frc.robot.subsystems.ClimberRightSubsystem;

public class ClimberToggleUpCommand extends Command {
  /** Creates a new ClimberToggleUpCommand. */
  ClimberLeftSubsystem m_climberLeftSubsystem;
  ClimberRightSubsystem m_climberRightSubsystem;
  String state = "down";
  boolean isfin = false;

  public ClimberToggleUpCommand(ClimberLeftSubsystem climberLeftSubsystem, ClimberRightSubsystem climberRightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberLeftSubsystem = climberLeftSubsystem;
    m_climberRightSubsystem = climberRightSubsystem;
    addRequirements(climberLeftSubsystem);
    addRequirements(climberRightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isfin = false; 
    m_climberRightSubsystem.setPositionRight(0.35);
    if (m_climberRightSubsystem.climbRightEncoder.getPosition() > Constants.Climber.climberTopSetpoint) {
      state = "up";
    } else {
      state = "down";
    }
  }

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(state == "down") {
			m_climberRightSubsystem.climbUPRight();
			m_climberLeftSubsystem.climbUPLeft();

			if(m_climberRightSubsystem.climbRightEncoder.getPosition() > Constants.Climber.climberTopSetpoint) {
				state = "up";
				isfin = true;
				isFinished();
			} else {
        m_climberRightSubsystem.climbDOWNRight();
        m_climberLeftSubsystem.climbDOWNLeft();
        if(m_climberLeftSubsystem.climbLeftEncoder.getPosition() < Constants.Climber.climberBottomSetpoint) {
          state = "down";
          isfin = true;
          isFinished();
			  }
		  }
		// SmartDashboard.putString("state", state);
    }

    if(state == "down") {
      m_climberRightSubsystem.climbUPRight();
      m_climberLeftSubsystem.climbUPLeft();
      if (m_climberRightSubsystem.climbRightEncoder.getPosition() > Constants.Climber.climberTopSetpoint) {
        state = "up";
        isfin = true;
        isFinished();
      }
    } else {
      m_climberRightSubsystem.climbDOWNRight();
      m_climberLeftSubsystem.climbDOWNLeft();
      if (m_climberRightSubsystem.climbRightEncoder.getPosition() < Constants.Climber.climberBottomSetpoint) {
        state = "down";
        isfin = true;
        isFinished();
      }
    }
    SmartDashboard.putString("state", state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberRightSubsystem.climbSTOP();
    m_climberLeftSubsystem.climbSTOP();
    m_climberRightSubsystem.setPositionRight(0.55);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfin;
  }
}
