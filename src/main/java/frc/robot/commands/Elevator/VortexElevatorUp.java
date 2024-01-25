// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Vortex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.VortexElevatorSub;

public class VortexElevatorUp extends Command {
  private VortexElevatorSub vortexElevatorSub;
  public VortexElevatorUp(VortexElevatorSub m_VortexElevatorSub) {
    vortexElevatorSub = m_VortexElevatorSub;
    addRequirements(vortexElevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vortexElevatorSub.elevatorUP();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vortexElevatorSub.elevatorSTOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
