// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VortexClimberSubsystem;

public class VortexClimberDown extends Command {
  private VortexClimberSubsystem m_vortexClimberSubsystem;

  public VortexClimberDown(VortexClimberSubsystem vortexClimber) {
    m_vortexClimberSubsystem /*help */ = vortexClimber;    
    addRequirements(m_vortexClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vortexClimberSubsystem.climbDOWN();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vortexClimberSubsystem.climbSTOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
