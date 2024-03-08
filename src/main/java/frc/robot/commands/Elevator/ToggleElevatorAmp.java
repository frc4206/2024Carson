// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToggleElevatorAmp extends Command {
  private ElevatorSubsystem m_elevator;
  private boolean toTop = false;
  public ToggleElevatorAmp(ElevatorSubsystem elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toTop = GlobalVariables.ampCounter % 2 == 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (toTop){
      m_elevator.GoToSetpoint(Constants.Elevator.elevatorTrapPosition);
    } else {
      m_elevator.GoToSetpoint(5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GlobalVariables.ampCounter++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
