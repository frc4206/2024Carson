// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberStopAllMotorsCommand.java
package frc.robot.commands.Climber;
========
package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj2.command.Command;
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbLeftToPosition.java
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberStopAllMotorsCommand.java
public class ClimberStopAllMotorsCommand extends Command {
  /** Creates a new ClimberStopAllMotorsCommand. */
  ClimberSubsystem m_climberSubsystem; 
  public ClimberStopAllMotorsCommand(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubsystem = climberSubsystem; 
    addRequirements(m_climberSubsystem);
========
public class ClimbLeftToPosition extends Command {
  private ClimberSubsystem m_climberLeft;
  private double m_desiredPosition;
  public ClimbLeftToPosition(ClimberSubsystem climberLeft, double desiredPosition) {
    m_climberLeft = climberLeft;
    m_desiredPosition = desiredPosition;
    addRequirements(m_climberLeft);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbLeftToPosition.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberStopAllMotorsCommand.java
    m_climberSubsystem.climbSTOP();
========
    m_climberLeft.climbToPosition(m_desiredPosition);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbLeftToPosition.java
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
