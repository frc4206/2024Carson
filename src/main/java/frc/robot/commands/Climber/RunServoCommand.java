// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RunServoCommand extends Command {
  
  public ClimberSubsystem m_climber;
  public double servoPosition;
  
  /** Creates a new RunServoCommand. */
  public RunServoCommand(ClimberSubsystem climber, double servoPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    servoPosition = servoPos;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setPosition(servoPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
