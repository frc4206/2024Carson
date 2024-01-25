// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VortexClimberSub;

public class VortexClimberUp extends Command {
  private VortexClimberSub vortexClimberSub;
  public VortexClimberUp(VortexClimberSub m_VortexClimberSub) {
    vortexClimberSub = m_VortexClimberSub;
    addRequirements(vortexClimberSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vortexClimberSub.climbUP();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vortexClimberSub.climbSTOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
