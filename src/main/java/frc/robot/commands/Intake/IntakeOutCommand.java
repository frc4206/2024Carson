// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOutCommand extends Command {
  /** Creates a new IntakeOut. */
  private IntakeSubsystem intakeMotor;
  public IntakeOutCommand(IntakeSubsystem m_intakeMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeMotor = m_intakeMotor; 
    addRequirements(intakeMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeMotor.intakeGo(-0.5);
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
