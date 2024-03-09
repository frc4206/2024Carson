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

public class ClimberSubsystem extends SubsystemBase {
	public static CANSparkFlex climbMotorRight;
	public static CANSparkFlex climbMotorLeft;
	private SparkPIDController climbPid;
	public RelativeEncoder climbEncoder;
	public XboxController climberControl;
	public PWM leftServo;
	public PWM rightServo;

	public ClimberSubsystem() {
		climbMotorRight = new CANSparkFlex(35, MotorType.kBrushless);
		climbMotorLeft = new CANSparkFlex(36, MotorType.kBrushless);
		climbMotorRight.setIdleMode(IdleMode.kBrake);
		climbMotorLeft.setIdleMode(IdleMode.kBrake);
		climbMotorRight.setInverted(false);
		climbMotorLeft.setInverted(true);

		leftServo = new PWM(1);
		rightServo = new PWM(0);

		climbEncoder = climbMotorRight.getEncoder();
		climbPid = climbMotorRight.getPIDController();
		
		climbPid.setFeedbackDevice(climbEncoder);
		climbPid.setP(Constants.Climber.climberkP);
		climbPid.setI(Constants.Climber.climberkI);
		climbPid.setIZone(Constants.Climber.climberkIZone);
		climbPid.setD(Constants.Climber.climberkD);
		climbPid.setSmartMotionMaxVelocity(Constants.Climber.climberMaxVelo, 0);
		climbPid.setSmartMotionMinOutputVelocity(-Constants.Climber.climberMaxVelo, 0);
		climbPid.setSmartMotionMaxAccel(Constants.Climber.climberMaxAcc, 0);
		climbPid.setSmartMotionAllowedClosedLoopError(Constants.Climber.climberAllowedError, 0);

		climbMotorRight.burnFlash();
		climbMotorLeft.burnFlash();
  	}

	/* Stop climber motors voids */
	public void climbStop(double servoPos) {
		climbMotorRight.set(0);
		setServoPosition(servoPos);
		climbMotorLeft.set(0);
	}

	/* Run climber motors up voids */
	public void climbUp(double servoPos, double motorSpeed) {
		setServoPosition(servoPos);
		climbMotorRight.set(motorSpeed);
		climbMotorLeft.set(motorSpeed);
	}

	/* Run climber motors down voids */
	public void climbDown(double servoPos) {
		setServoPosition(servoPos);
		climbMotorLeft.set(-0.2);
		climbMotorRight.set(-0.2);
	}

	/* PID setpoint set */
	public void GoToSetpoint(double setpoint) { 
		climbPid.setReference(setpoint, ControlType.kPosition, 0); 
	}

	/* Servo set positions methods */
	public void setServoPosition(double servoPos) {
		leftServo.setPosition(servoPos);
		rightServo.setPosition(servoPos);
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
