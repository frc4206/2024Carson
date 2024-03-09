// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<< HEAD
package frc.robot.commands.Climber.ClimberLeft;
=======
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimbUpRight.java
package frc.robot.commands.Climber.ClimberRight;
========
package frc.robot.commands.Climber.ClimberLeft;
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbUpLeft.java
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

<<<<<<< HEAD
=======
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimbUpRight.java
public class ClimbUpRight extends Command {
  private ClimberSubsystem m_climberRight;
  private double startTime = 0;
  private double currTime = 0;
  public ClimbUpRight(ClimberSubsystem climberRight) {
    m_climberRight = climberRight;
    addRequirements(m_climberRight);
========
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
public class ClimbUpLeft extends Command {
  private ClimberSubsystem m_climberLeft;
  private double startTime = 0;
  private double currTime = 0;
  public ClimbUpLeft(ClimberSubsystem climberLeft) {
    m_climberLeft = climberLeft;
    addRequirements(m_climberLeft);
<<<<<<< HEAD
=======
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbUpLeft.java
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberDownCommand.java
    //m_climber.setPositionBoth(1, Constants.Climber.servoPosRightDisEngage);
    currtime = Timer.getFPGATimestamp() - start;
    if (currtime > 0.5) {
      m_climber.climbDOWN();
=======
=======
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimbUpRight.java
    m_climberRight.setPosition(Constants.Climber.servoPosRightDisEngage);
    currTime = Timer.getFPGATimestamp() - startTime;
    if (currTime > 0.5) {
      m_climberRight.climbToDuty(0.15);
========
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
    m_climberLeft.setPosition(Constants.Climber.servoPosLeftDisEngage);
    currTime = Timer.getFPGATimestamp() - startTime;
    if (currTime > 0.5) {
      m_climberLeft.climbToDuty(0.15);
<<<<<<< HEAD
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbUpLeft.java
=======
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbUpLeft.java
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<< HEAD
<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberDownCommand.java
    m_climber.climbSTOP();
    //m_climber.setPosition(Constants.Climber.servoPosEngage);
=======
    m_climberLeft.climbToDuty(0);
    m_climberLeft.setPosition(Constants.Climber.servoPosLeftEngage);
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbUpLeft.java
=======
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimbUpRight.java
    m_climberRight.setPosition(Constants.Climber.servoPosRightEngage);
    m_climberRight.climbToDuty(0);
========
    m_climberLeft.climbToDuty(0);
    m_climberLeft.setPosition(Constants.Climber.servoPosLeftEngage);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbUpLeft.java
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
