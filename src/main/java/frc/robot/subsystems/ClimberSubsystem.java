// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.commands.Climber.RunServosCommand;
import frc.robot.commands.Climber.ClimberLeft.RunServoLeftCommand;
import frc.robot.commands.Climber.ClimberRight.RunServoRightCommand;

public class ClimberSubsystem extends SubsystemBase {
	public static CANSparkFlex climbMotor = new CANSparkFlex(Constants.Climber.climberRightLeadID, MotorType.kBrushless);
	private SparkPIDController climbPid;
	public RelativeEncoder climbEncoder;
	public XboxController climberControl;
	private double motorRunSpeed;

	static PWM servo;

	public ClimberSubsystem(int climbMotorId, boolean isInverted, int servoChannel) {
		climbMotor = new CANSparkFlex(climbMotorId, MotorType.kBrushless);
		climbMotor.setIdleMode(IdleMode.kBrake);
		climbMotor.setInverted(isInverted);

		climbEncoder = climbMotor.getEncoder();
		climbPid = climbMotor.getPIDController();
		
		climbPid.setFeedbackDevice(climbEncoder);
		climbPid.setP(Constants.Climber.climberkP);
		climbPid.setI(Constants.Climber.climberkI);
		climbPid.setIZone(Constants.Climber.climberkIZone);
		climbPid.setD(Constants.Climber.climberkD);
		climbPid.setSmartMotionMaxVelocity(Constants.Climber.climberMaxVelo, 0);
		climbPid.setSmartMotionMinOutputVelocity(-Constants.Climber.climberMaxVelo, 0);
		climbPid.setSmartMotionMaxAccel(Constants.Climber.climberMaxAcc, 0);
		climbPid.setSmartMotionAllowedClosedLoopError(Constants.Climber.climberAllowedError, 0);

		climbMotor.burnFlash();

		servo = new PWM(servoChannel);
  	}

	/* Stop climber motors voids */
	public void climbSTOP() {
		climbMotor.set(0);
		setServoPosition(Constants.Climber.servoPosLeftEngage);
	}

	/* Run climber motors up voids */
	public void climberUP() {
		setServoPosition(Constants.Climber.servoPosLeftDisEngage);

		// Motor speed depends on which trigger is held down more.
		climbMotor.set(climberControl.getLeftTriggerAxis());
	}

	/* Run climber motors down voids */
	public void climbDOWN() {
		setServoPosition(Constants.Climber.servoPosLeftDisEngage);
		climbMotor.set(-0.2);
		climbMotor.set(0.2);
	}

	/* PID setpoint set */
	public void GoToSetpoint(double setpoint) { 
		climbPid.setReference(setpoint, ControlType.kPosition, 0); 
	}

	/* Servo set positions methods */
	public void setServoPosition(double servoPos) {
		servo.setPosition(servoPos);
	}

	@Override
	public void periodic() {
		// if(TopClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(0);
		// }
		// if(BottomClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(Constants.Climber.climberResetPosition);
		// }

		// SmartDashboard.putNumber("Climber Right Position", climbRightLeadEncoder.getPosition());
		// SmartDashboard.putNumber("Climber Left Position", climbLeftFollowEncoder.getPosition());

		// SmartDashboard.putNumber("servo Right", servoRight.getPosition());
		// SmartDashboard.putNumber("servo Left", servoLeft.getPosition());
	}
}
