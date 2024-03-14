// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.FlywheelSubsystem;

public class ToggleShooterToVelocity extends Command {
  private FlywheelSubsystem m_flywheel;
  private double m_desiredVelocity;
  private boolean toVelo;
  private boolean isFinished = false;
  public ToggleShooterToVelocity(FlywheelSubsystem flywheel, double desiredVelocity) {
    m_flywheel = flywheel;
    m_desiredVelocity = desiredVelocity;
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toVelo = GlobalVariables.veloCounter % 2 == 0;
    if (toVelo){
      m_flywheel.setVelocity(m_desiredVelocity);
    } else {
      m_flywheel.percentShooter(0);
    }
    isFinished = true;
    isFinished();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GlobalVariables.veloCounter++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
