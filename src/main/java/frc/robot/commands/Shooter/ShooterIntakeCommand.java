// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterIntakeCommand extends Command {
  /** Creates a new ShooterIntakeCommand. */
  private IntakeSubsystem m_intakeSubsystem; 
  private boolean finished = false; 
  private FlywheelSubsystem m_flywheelSubsystem; 

  public ShooterIntakeCommand(IntakeSubsystem intake, FlywheelSubsystem flywheel) {
    this.m_flywheelSubsystem = flywheel; 
    this.m_intakeSubsystem = intake; 
    addRequirements(intake);
    addRequirements(flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intakeSubsystem.intakeBeamBreakValue == true) {
      m_flywheelSubsystem.upperFlyMotor.set(0.2); 
    }
    if(m_flywheelSubsystem.shooterBeamBreak.get() == true) {
      m_flywheelSubsystem.upperFlyMotor.set(0.0);
    }
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
