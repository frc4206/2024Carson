// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;
import frc.robot.subsystems.ClimberLeftSubsystem;
import frc.robot.subsystems.ClimberRightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberRunRightCommand extends Command {
  /** Creates a new ClimberRunRightCommand. */
  ClimberRightSubsystem m_ClimberRightSubsystem; 

  public ClimberRunRightCommand(ClimberRightSubsystem climberRightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberRightSubsystem = climberRightSubsystem; 
    addRequirements(climberRightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimberRightSubsystem.RunRight();
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
