// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ClimberSubsystem;

public class ToggleClimbToPosition extends Command {
  private ClimberSubsystem m_leftClimber;
  private ClimberSubsystem m_rightClimber;
  private boolean toFirst;
  private boolean isFinished = false;
  private double m_firstLeftPosition;
  private double m_secondLeftPosition;
  private double m_firstRightPosition;
  private double m_secondRightPosition;
  public ToggleClimbToPosition(ClimberSubsystem leftClimber, ClimberSubsystem rightClimber, double firstLeftPosition, double secondLeftPosition, double firstRightPosition, double secondRightPosition) {
    m_leftClimber = leftClimber;
    m_rightClimber = rightClimber;
    m_firstLeftPosition = firstLeftPosition;
    m_firstRightPosition = firstRightPosition;
    m_secondLeftPosition = secondLeftPosition;
    m_secondRightPosition = secondRightPosition;
    addRequirements(m_leftClimber, m_rightClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toFirst = GlobalVariables.climbCounter % 2 == 0;
    if (toFirst){
      m_leftClimber.climbToPosition(m_firstLeftPosition);
      m_rightClimber.climbToPosition(m_firstRightPosition);
    } else {
      m_leftClimber.climbToPosition(m_secondLeftPosition);
      m_rightClimber.climbToPosition(m_secondRightPosition);
    }
    isFinished = true;
    isFinished();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GlobalVariables.climbCounter++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
