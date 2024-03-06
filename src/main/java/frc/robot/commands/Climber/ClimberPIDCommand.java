// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
//import frc.robot.subsystems.VortexClimberSubsystem;
import frc.robot.subsystems.ClimberLeftSubsystem;
import frc.robot.subsystems.ClimberRightSubsystem;

public class ClimberPIDCommand extends Command {
  
  public ClimberLeftSubsystem m_vortexClimberLeftSubsystem;
  public ClimberRightSubsystem m_vortexClimberRightSubsystem;

  /** The VortexClimberPID command can be called whenever we need to call the motor controller to climb the chain. */
  public ClimberPIDCommand(ClimberRightSubsystem vortexClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vortexClimberRightSubsystem = vortexClimber;
    addRequirements(vortexClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vortexClimberRightSubsystem.GoToSetpoint(Constants.Climber.climberGoToSetPoint);
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