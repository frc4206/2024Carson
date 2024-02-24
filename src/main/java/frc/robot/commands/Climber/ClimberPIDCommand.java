// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.VortexClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberPIDCommand extends Command {
  private ClimberSubsystem m_vortexClimberSubsystem;
  private double m_setpoint;
  public ClimberPIDCommand(ClimberSubsystem vortexClimber, double setpoint) {
    m_vortexClimberSubsystem = vortexClimber;
    m_setpoint = setpoint;
    addRequirements(vortexClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vortexClimberSubsystem.GoToSetpoint(m_setpoint);
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
