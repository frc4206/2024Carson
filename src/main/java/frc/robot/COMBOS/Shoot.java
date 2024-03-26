// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.COMBOS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Shoot extends Command {
  private ConveyorSubsystem m_conveyor;
  private IntakeSubsystem m_intake;
  private double m_desiredConveyorDuty;
  private double m_desiredIntakeDuty;
  public Shoot(ConveyorSubsystem conveyor, IntakeSubsystem intake) {
    m_conveyor = conveyor;
    m_intake = intake;
    addRequirements(m_conveyor, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (GlobalVariables.Flywheel.toAmpVelo){
      m_desiredConveyorDuty = 0.45;
      m_desiredIntakeDuty = 0.5;
    } else {
      m_desiredConveyorDuty = 0.9;
      m_desiredIntakeDuty = 1;
    }

    m_conveyor.conveyorToDuty(m_desiredConveyorDuty);
    m_intake.intakeToDuty(m_desiredIntakeDuty);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.conveyorToDuty(0);
    m_intake.intakeToDuty(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
