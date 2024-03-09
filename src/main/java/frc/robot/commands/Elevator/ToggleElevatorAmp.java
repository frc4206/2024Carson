// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimberUpRightCommand.java
package frc.robot.commands.Climber.ClimberRight;
========
package frc.robot.commands.Elevator;
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Elevator/ToggleElevatorAmp.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ElevatorSubsystem;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimberUpRightCommand.java
public class ClimberUpRightCommand extends Command {
private ClimberSubsystem m_climberRight;
  double start = 0;
  double currtime = 0;

  public ClimberUpRightCommand(ClimberSubsystem climber_R) {
    m_climberRight = climber_R;
    addRequirements(m_climberRight);
========
public class ToggleElevatorAmp extends Command {
  private ElevatorSubsystem m_elevator;
  private boolean toTop = false;
  public ToggleElevatorAmp(ElevatorSubsystem elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Elevator/ToggleElevatorAmp.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toTop = GlobalVariables.ampCounter % 2 == 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimberUpRightCommand.java
    m_climberRight.setPositionRight(Constants.Climber.servoPosRightDisEngage);
    currtime = Timer.getFPGATimestamp() - start;
    if (currtime > 0.2) {
      m_climberRight.climbUPRight();
========
    if (toTop){
      m_elevator.GoToSetpoint(Constants.Elevator.elevatorTrapPosition);
    } else {
      m_elevator.GoToSetpoint(5);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Elevator/ToggleElevatorAmp.java
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRight/ClimberUpRightCommand.java
    m_climberRight.climbSTOP();
    m_climberRight.setPositionRight(Constants.Climber.servoPosEngage);
========
    GlobalVariables.ampCounter++;
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Elevator/ToggleElevatorAmp.java
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
