// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ClimberSubsystem;

public class ToggleClimberFar extends Command {
  private ClimberSubsystem m_climber;

  private boolean servoDisengaged = false;
	private long startServoTime = 0;
	private long currentTime = 0;
  private boolean toTop = false;
  private boolean isFinished = false;
  
  public ToggleClimberFar(ClimberSubsystem climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toTop = GlobalVariables.Climber.yPressed % 2 == 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		if (toTop) {
			m_climber.disengageServo();
			if (!servoDisengaged) {
				startServoTime = System.currentTimeMillis();
				servoDisengaged = true;
			}
		} else {
			m_climber.engageServo();
			servoDisengaged = false;
		}

    currentTime = System.currentTimeMillis();

    if (servoDisengaged){
      if (currentTime - startServoTime <= Constants.Climber.disengageDurationMilliseconds) {
        m_climber.climberToDuty(0);
      } else if (currentTime - startServoTime > Constants.Climber.disengageDurationMilliseconds){
        if (toTop){
          m_climber.climberToPosition(Constants.Climber.farPosition);
          isFinished = true;
          isFinished();
        }
      }
    } else {
      m_climber.climberToPosition(Constants.Climber.zeroPosition);
      isFinished = true;
      isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GlobalVariables.Climber.yPressed++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
