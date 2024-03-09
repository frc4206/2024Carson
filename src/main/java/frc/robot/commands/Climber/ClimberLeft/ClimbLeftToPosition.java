// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<< HEAD
package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberPIDCommand.java
import frc.robot.Constants;
//import frc.robot.subsystems.VortexClimberSubsystem;
import frc.robot.subsystems.ClimberLeftSubsystem;
import frc.robot.subsystems.ClimberRightSubsystem;

public class ClimberPIDCommand extends Command {
  public ClimberLeftSubsystem m_climberLeftSubsystem;
  public ClimberRightSubsystem m_climberRightSubsystem;
  public double setpoint;

  /** The VortexClimberPID command can be called whenever we need to call the motor controller to climb the chain. */
  public ClimberPIDCommand(ClimberRightSubsystem climbRight, ClimberLeftSubsystem climbLeft, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberRightSubsystem = climbRight;
    m_climberLeftSubsystem = climbLeft;
    this.setpoint = setpoint;
    addRequirements(climbRight);
    addRequirements(climbLeft);
=======
import frc.robot.subsystems.ClimberSubsystem;

=======
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
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
public class ClimbLeftToPosition extends Command {
  private ClimberSubsystem m_climberLeft;
  private double m_desiredPosition;
  public ClimbLeftToPosition(ClimberSubsystem climberLeft, double desiredPosition) {
    m_climberLeft = climberLeft;
    m_desiredPosition = desiredPosition;
    addRequirements(m_climberLeft);
<<<<<<< HEAD
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbLeftToPosition.java
=======
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbLeftToPosition.java
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberPIDCommand.java
    m_climberRightSubsystem.GoToSetpoint(Constants.Climber.climberTopSetpoint);
=======
    m_climberLeft.climbToPosition(m_desiredPosition);
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbLeftToPosition.java
=======
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberStopAllMotorsCommand.java
    m_climberSubsystem.climbSTOP();
========
    m_climberLeft.climbToPosition(m_desiredPosition);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Climber/ClimberLeft/ClimbLeftToPosition.java
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
<<<<<<< HEAD
}
=======
}
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
