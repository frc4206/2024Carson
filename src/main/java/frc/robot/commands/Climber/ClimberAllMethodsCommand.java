// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;
import frc.robot.subsystems.ClimberLeftSubsystem;
import frc.robot.subsystems.ClimberRightSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberAllMethodsCommand extends Command {
  /** Creates a new ClimberAllMethodsCommand. */

  public static XboxController joystick; 
  ClimberLeftSubsystem m_ClimberLeftSubsystem; 
  ClimberRightSubsystem m_ClimberRightSubsystem;
  
  public ClimberAllMethodsCommand(ClimberLeftSubsystem climberLeftSubsystem, ClimberRightSubsystem climberRightSubsystem ,  XboxController joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberLeftSubsystem = climberLeftSubsystem;
    m_ClimberRightSubsystem = climberRightSubsystem;
    joystick = joy; 
    addRequirements(climberLeftSubsystem);
    addRequirements(climberRightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (Math.abs(joystick.getRawAxis(5)) > 0.1) {
        m_ClimberRightSubsystem.RunRight();
      }
      else if (Math.abs(joystick.getRawAxis(1)) > 0.1) {
        m_ClimberLeftSubsystem.RunLeft(); 
      }
      else if (Math.abs(joystick.getRightTriggerAxis()) > 0.1) {
        m_ClimberLeftSubsystem.RunBothMotorsDown();
        m_ClimberRightSubsystem.RunBothMotorsDown();
      }
      else if (Math.abs(joystick.getLeftTriggerAxis()) > 0.1) {
        m_ClimberLeftSubsystem.RunBothMotorsUp();
        m_ClimberRightSubsystem.RunBothMotorsUp();
      }
      else {
        m_ClimberLeftSubsystem.climbSTOP();
        m_ClimberRightSubsystem.climbSTOP();
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
