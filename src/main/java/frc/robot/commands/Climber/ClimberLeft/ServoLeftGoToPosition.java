// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ServoLeftGoToPosition extends Command {
  private ClimberSubsystem m_climberLeft;
  private double m_leftServoPosition;
  public ServoLeftGoToPosition(ClimberSubsystem climberLeft, double leftServoPosition) {
    m_climberLeft = climberLeft;
    m_leftServoPosition = leftServoPosition;
    addRequirements(m_climberLeft);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberLeft.setPosition(m_leftServoPosition);
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
