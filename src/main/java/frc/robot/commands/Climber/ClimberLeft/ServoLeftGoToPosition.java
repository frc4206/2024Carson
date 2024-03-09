// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/ServoLeftGoToPosition.java
package frc.robot.commands.Climber.ClimberLeft;
========
package frc.robot.commands.Climber.ClimberRight;
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberRight/ServoRightGoToPosition.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/ServoLeftGoToPosition.java
public class ServoLeftGoToPosition extends Command {
  private ClimberSubsystem m_climberLeft;
  private double m_leftServoPosition;
  public ServoLeftGoToPosition(ClimberSubsystem climberLeft, double leftServoPosition) {
    m_climberLeft = climberLeft;
    m_leftServoPosition = leftServoPosition;
    addRequirements(m_climberLeft);
========
public class ServoRightGoToPosition extends Command {
  private ClimberSubsystem m_climberRight;
  private double m_rightServoPosition;
  public ServoRightGoToPosition(ClimberSubsystem climberRight, double rightServoPosition) {
    m_climberRight = climberRight;
    m_rightServoPosition = rightServoPosition;
    addRequirements(m_climberRight);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberRight/ServoRightGoToPosition.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/ServoLeftGoToPosition.java
    m_climberLeft.setPosition(m_leftServoPosition);
========
    m_climberRight.setPosition(m_rightServoPosition);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberRight/ServoRightGoToPosition.java
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
