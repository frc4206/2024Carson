// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbLeftSubsystem;

public class ClimbDownLeftCommand extends Command {
  private ClimbLeftSubsystem m_climberLeft;
  private double startTime = 0;
  private double currTime = 0;
  public ClimbDownLeftCommand(ClimbLeftSubsystem climberLeft) {
    m_climberLeft = climberLeft;
    addRequirements(m_climberLeft);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberLeft.setPosition(Constants.Climber.servoPosLeftDisEngage);
    currTime = Timer.getFPGATimestamp() - startTime;
    if (currTime > 0.5) {
      m_climberLeft.climbDOWN();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberLeft.setPosition(Constants.Climber.servoPosLeftEngage);
    m_climberLeft.climbSTOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}