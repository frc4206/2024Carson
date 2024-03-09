// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberRight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbRightSubystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbRightToPosition extends Command {
  private ClimberSubsystem m_climberRight;
  private double m_desiredPosition;
  public ClimbRightToPosition(ClimberSubsystem climberRight, double desiredPosition) {
    m_climberRight = climberRight;
    m_desiredPosition = desiredPosition;
    addRequirements(m_climberRight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_climberRight.climbToPosition(m_desiredPosition);
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
