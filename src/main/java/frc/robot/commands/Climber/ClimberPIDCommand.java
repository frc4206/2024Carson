// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberPIDCommand extends Command {
  
  public ClimberSubsystem m_vortexClimberSubsystem;

  /** The VortexClimberPID command can be called whenever we need to call the motor controller to climb the chain. */
  public ClimberPIDCommand(ClimberSubsystem vortexClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vortexClimberSubsystem = vortexClimber;
    addRequirements(vortexClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vortexClimberSubsystem.GoToSetpoint(-100);
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
