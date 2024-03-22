// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpBar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.AmpBarSubsystem;

public class AmpBarToAmp extends Command {
  private AmpBarSubsystem m_ampBar;
  private boolean pastFour = false;
  private boolean isFinished = false;
  public AmpBarToAmp(AmpBarSubsystem ampBar) {
    m_ampBar = ampBar;
    addRequirements(m_ampBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (GlobalVariables.AmpBar.ampBarPosition < 4){
      isFinished = true;
      isFinished();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!pastFour){
      if (GlobalVariables.AmpBar.ampBarPosition > 4){
        m_ampBar.ampBarToPosition(4);
      } else {
        pastFour = true;
      }
    } else {
      if (!GlobalVariables.AmpBar.ampBarAtZero){
        m_ampBar.ampBarToDuty(-0.05);
      } else {
        m_ampBar.ampBarToDuty(0);
        m_ampBar.resetAmpBarEncoder();
        isFinished = true;
        isFinished();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
