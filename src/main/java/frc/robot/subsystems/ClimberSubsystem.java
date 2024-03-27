// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
	private CANSparkFlex climberMotor = new CANSparkFlex(Constants.Climber.climberMotorID, MotorType.kBrushless);
	private RelativeEncoder climberEncoder = climberMotor.getEncoder();
	private SparkPIDController climberPIDController = climberMotor.getPIDController();
	SparkConfig climberConfig;

	PWM servo = new PWM(Constants.Climber.servoID);

	private XboxController controller;
	private int motorAxis;

	private boolean servoDisengaged = false;
	private long startServoTime = 0;
	private long disengageDurationMilliseconds = 180;

	public ClimberSubsystem() {
		climberConfig = Constants.Climber.climberConfig;
		climberConfig.configureController(climberMotor, climberEncoder, climberPIDController);
		climberConfig.applyAllConfigurations();
	}

	public boolean setupController(XboxController controller, int axis) {
		if (controller == null)
			return false;
		this.controller = controller;
		this.motorAxis = axis;
		return true;
	}

	public double squareDeadzone(double val, double deadzone) {
		double deadzoned = (Math.abs(val) >= deadzone ? map(Math.abs(val), deadzone, 1.0d, 0.0d, 1.0d) : 0.0d);
		return val >= 0.0d ? deadzoned : -deadzoned;
	}

	public double quadratic(double val){
		if (val < 0){
			return -(val * val);
		} else {
			return val * val;
		}
	}

	public double cubic(double val) {
		return val * val * val;
	}

	public double map(double val, double inMin, double inMax, double outMin, double outMax) {
		return ((val - inMin) * (outMax - outMin) / (inMax - inMin)) + outMin;
	}

	@Override
	public void periodic() {
		double motorSetSpeed = 0.0d;

		double rightTriggerSpeed = quadratic(-this.controller.getRightTriggerAxis());
		double leftTriggerSpeed = quadratic(this.controller.getLeftTriggerAxis());
		double joystickSpeed = -this.controller.getRawAxis(this.motorAxis);
		joystickSpeed = squareDeadzone(joystickSpeed, Constants.OperatorConstants.joystickDeadzone);

		// ignore if both right and left trigger are pressed
		if (rightTriggerSpeed > 0.0d && leftTriggerSpeed > 0.0d) {
			climberMotor.set(0.0d); // safe to zero for safety
			return; // return early, multiple inputs are unclear so ignore
		}

		// by this point, either one is these is NOT zero or both are
		if (leftTriggerSpeed > 0.0d) {
			motorSetSpeed = -leftTriggerSpeed;
		}
		if (rightTriggerSpeed > 0.0d) {
			motorSetSpeed = rightTriggerSpeed; // opposite direction
		}

		// if nothing set, safe to use joystick input
		if (motorSetSpeed == 0.0d && joystickSpeed != 0.0d) {
			joystickSpeed = cubic(joystickSpeed); // apply response curve to user input
			motorSetSpeed = joystickSpeed;
		}

		// if spinning against the pawl, open the pawl servo
		if (motorSetSpeed < 0.0d) {
			servo.setPulseTimeMicroseconds(Constants.Climber.servoDisengage);
			//servo.setPosition(this.disengageServoPos);
			// start a timer if we are just now pressing the button
			if (!this.servoDisengaged) {
				this.startServoTime = System.currentTimeMillis();
				this.servoDisengaged = true;
			}
		} else {
			// spinning with from pawl
			servo.setPulseTimeMicroseconds(Constants.Climber.servoEngage);
			//servo.setPosition(this.engageServoPos);
			this.servoDisengaged = false;
		}

		long currentTime = System.currentTimeMillis();

		// if not enough time has passed for the servos to disengage
		// then we should not start moving the motors yetq
		if (currentTime - startServoTime <= this.disengageDurationMilliseconds && this.servoDisengaged) {
			motorSetSpeed = 0.0d;
		}

		// set motor
		climberMotor.set(motorSetSpeed);

		SmartDashboard.putNumber("climberCurrent", climberMotor.getOutputCurrent());
	}
}