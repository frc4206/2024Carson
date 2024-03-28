// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberToDuty extends Command {
  private ClimberSubsystem m_climber;
  private Supplier<Double> m_axisSupplier;
  private double m_axisValue = 0;

  private boolean servoDisengaged = false;
	private long startServoTime = 0;
	private long currentTime = 0;
	private double joystickSpeed = 0;	
	private double motorSetSpeed = 0;

  public ClimberToDuty(ClimberSubsystem climber, Supplier<Double> axisSupplier) {
    m_climber = climber;
    m_axisSupplier = axisSupplier;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_axisValue = m_axisSupplier.get();
		joystickSpeed = m_climber.squareDeadzone(m_axisValue, Constants.OperatorConstants.joystickDeadzone);
		motorSetSpeed = 0.0d;

    if (joystickSpeed != 0.0d) {
			joystickSpeed = m_climber.cubic(joystickSpeed);
			motorSetSpeed = joystickSpeed;
		}

		if (motorSetSpeed < 0.0d) {
			m_climber.disengageServo();
			if (!servoDisengaged) {
				startServoTime = System.currentTimeMillis();
				servoDisengaged = true;
			}
		} else {
			m_climber.engageServo();
			servoDisengaged = false;
		}

    currentTime = System.currentTimeMillis();

		if (currentTime - startServoTime <= Constants.Climber.disengageDurationMilliseconds && servoDisengaged) {
			motorSetSpeed = 0.0d;
		}

		m_climber.climberToDuty(motorSetSpeed);
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
