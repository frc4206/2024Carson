// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberToggleUpCommand extends Command {
  /** Creates a new ClimberToggleUpCommand. */
  ClimberSubsystem m_climberSubsystem;
  String state = "down";
  boolean isfin = false;
  public ClimberToggleUpCommand(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSubsystem.setPosition(0.3);
    if (m_climberSubsystem.climbLeadEncoder.getPosition() > Constants.Climber.climberTopSetpoint) {
      state = "up";
    } else {
      state = "down";
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (state == "down") {
      m_climberSubsystem.climbUP();
      if (m_climberSubsystem.climbLeadEncoder.getPosition() > Constants.Climber.climberTopSetpoint) {
        state = "up";
        isfin = true;
        isFinished();
      }
    }else {
      m_climberSubsystem.climbDOWN();
      if (m_climberSubsystem.climbLeadEncoder.getPosition() < Constants.Climber.climberBottomSetpoint) {
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
    m_climberSubsystem.climbSTOP();
    m_climberSubsystem.setPosition(0.6);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfin;
  }
}