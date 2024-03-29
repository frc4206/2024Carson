// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.ClimberRight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimbDownRight.java
public class ClimbDownRight extends Command {
  private ClimberSubsystem m_climberRight;
  private double startTime = 0;
  private double currTime = 0;
  public ClimbDownRight(ClimberSubsystem climberRight) {
========
public class ClimbUpRight extends Command {
  private ClimberSubsystem m_climberRight;
  private double startTime = 0;
  private double currTime = 0;
  public ClimbUpRight(ClimberSubsystem climberRight) {
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimbUpRight.java
    m_climberRight = climberRight;
    addRequirements(m_climberRight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberRight.setPosition(Constants.Climber.servoPosRightDisEngage);
    currTime = Timer.getFPGATimestamp() - startTime;
    if (currTime > 0.5) {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimbDownRight.java
      m_climberRight.climbToDuty(-0.15);
========
      m_climberRight.climbToDuty(0.15);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimbUpRight.java
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberRight.setPosition(Constants.Climber.servoPosRightEngage);
    m_climberRight.climbToDuty(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
