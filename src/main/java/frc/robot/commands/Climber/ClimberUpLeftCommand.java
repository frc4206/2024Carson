// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberLeftSubsystem;
import frc.robot.subsystems.ClimberRightSubsystem;

public class ClimberUpLeftCommand extends Command {

  private ClimberLeftSubsystem m_vortexClimberLeftSubsystem;
  private ClimberRightSubsystem m_vortexClimberRightSubsystem;

  /** The VortexClimberUp command can be called whenever we need to go up the chain. */
  public ClimberUpLeftCommand(ClimberLeftSubsystem vortexLeftClimber, ClimberRightSubsystem vortexRightClimber) {
    m_vortexClimberLeftSubsystem = vortexLeftClimber;
    m_vortexClimberRightSubsystem = vortexRightClimber;
    addRequirements(vortexLeftClimber);
    addRequirements(vortexRightClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vortexClimberRightSubsystem.setPositionRight(Constants.Climber.servoPosRightDisEngage);
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vortexClimberLeftSubsystem.climbUPLeft();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vortexClimberLeftSubsystem.climbSTOP();
    m_vortexClimberRightSubsystem.setPositionRight(Constants.Climber.servoPosRightDisEngage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // public void testForLoops() {
  //   public static int[] values = {0, 4, 6, 2};

  //   for(int i = 0; i < values.length; i++) {
  //     System.out.println(i);
  //   }

  //   for(int value : values) {
  //     System.out.println(value);
  //   }
  // }
  

}
