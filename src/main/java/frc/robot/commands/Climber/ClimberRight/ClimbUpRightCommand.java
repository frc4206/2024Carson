// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberRight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbRightSubystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUpRightCommand extends Command {
  private ClimberSubsystem m_climberRight;
  private double startTime = 0;
  private double currTime = 0;
  public ClimbUpRightCommand(ClimberSubsystem climberRight) {
    m_climberRight = climberRight;
    addRequirements(m_climberRight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberRight.setPosition(Constants.Climber.servoPosRightDisEngage);
    currTime = Timer.getFPGATimestamp() - startTime;
    if (currTime > 0.5) {
      m_climberRight.climberUP();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberRight.setPosition(Constants.Climber.servoPosRightEngage);
    m_climberRight.climbSTOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
