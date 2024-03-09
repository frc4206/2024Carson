// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimberUpLeftCommand.java
public class ClimberUpLeftCommand extends Command {
	private ClimberSubsystem m_climberLeft;
  double start = 0;
  double currtime = 0;

  public ClimberUpLeftCommand(ClimberSubsystem climber) {
    m_climberLeft = climber;
========
public class ClimbDownLeft extends Command {
  private ClimberSubsystem m_climberLeft;
  private double startTime = 0;
  private double currTime = 0;
  public ClimbDownLeft(ClimberSubsystem climberLeft) {
    m_climberLeft = climberLeft;
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbDownLeft.java
    addRequirements(m_climberLeft);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimberUpLeftCommand.java
    m_climberLeft.setPositionLeft(Constants.Climber.servoPosRightDisEngage);
    currtime = Timer.getFPGATimestamp() - start;
    if (currtime > 0.2) {
      m_climberLeft.climbUPLeft();
========

    System.out.println("Presssed the left command button");
    m_climberLeft.setPosition(Constants.Climber.servoPosLeftDisEngage);
    currTime = Timer.getFPGATimestamp() - startTime;
    if (currTime > 0.15d) {
      m_climberLeft.climbToDuty(-0.15);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbDownLeft.java
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimberUpLeftCommand.java
    m_climberLeft.climbSTOP();
    m_climberLeft.setPositionLeft(Constants.Climber.servoPosEngage);
========
    m_climberLeft.setPosition(Constants.Climber.servoPosLeftEngage);
    m_climberLeft.climbToDuty(0);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbDownLeft.java
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
