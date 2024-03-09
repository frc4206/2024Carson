// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUpLeftCommand extends Command {
  private ClimberSubsystem m_climberLeft;
  private double startTime = 0;
  private double currTime = 0;
  private XboxController controller;
  public ClimbUpLeftCommand(ClimberSubsystem climberLeft, int controllerPort) {
    m_climberLeft = climberLeft;
    controller = new XboxController(controllerPort);
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
    //m_climberLeft.setServoPosition(Constants.Climber.servoPosLeftDisEngage);
    currTime = Timer.getFPGATimestamp() - startTime;
    if (currTime > 0.2) {
      m_climberLeft.climbUp(Constants.Climber.servoPosLeftDisEngage, controller.getLeftTriggerAxis());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberLeft.climbStop(Constants.Climber.servoPosLeftEngage);
   // m_climberLeft.setServoPosition(Constants.Climber.servoPosLeftEngage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
