// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpLeftCommand extends Command {
	private ClimberSubsystem m_climberLeft;
  double start = 0;
  double currtime = 0;

  public ClimberUpLeftCommand(ClimberSubsystem climber) {
    m_climberLeft = climber;
    addRequirements(m_climberLeft);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberLeft.setPositionLeft(Constants.Climber.servoPosRightDisEngage);
    currtime = Timer.getFPGATimestamp() - start;
    if (currtime > 0.2) {
      m_climberLeft.climbUPLeft();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberLeft.climbSTOP();
    m_climberLeft.setPositionLeft(Constants.Climber.servoPosEngage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
