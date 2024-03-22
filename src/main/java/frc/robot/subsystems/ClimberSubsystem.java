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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.sparkConfig.ControllerConfig;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
	private CANSparkFlex climberMotor;
	private RelativeEncoder climberEncoder;
	private SparkPIDController climberPIDController;
	SparkConfig climberConfig;
	ControllerConfig controllerConfig;
	PWM servo;

	private XboxController controller;
	private int motorAxis;

	public int engageServoPos = 0;
	public int disengageServoPos = 0;
	private boolean servoDisengaged = false;
	private long startServoTime = 0;
	private long disengageDurationMilliseconds = 180;

	public enum SIDE {
		LEFT,
		RIGHT
	}

	public ClimberSubsystem(int motorID, int servoID, SIDE side) {
		climberMotor = new CANSparkFlex(motorID, MotorType.kBrushless);
		climberEncoder = climberMotor.getEncoder();
		climberPIDController = climberMotor.getPIDController();
		servo = new PWM(servoID);
		
		if (side == SIDE.LEFT){
			climberConfig = Constants.Climber.climberLeftConfig;
		} else if (side == SIDE.RIGHT){
			climberConfig = Constants.Climber.climberRightConfig;
		}
		climberConfig.configureController(climberMotor, climberEncoder, climberPIDController);
		climberConfig.applyConfigurations();
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

	public double quadratic(double val) {
		return val * val * val;
	}

	public double map(double val, double inMin, double inMax, double outMin, double outMax) {
		return ((val - inMin) * (outMax - outMin) / (inMax - inMin)) + outMin;
	}

	@Override
	public void periodic() {
		double motorSetSpeed = 0.0d;

		// trigger input
		double rightTriggerSpeed = this.controller.getRightTriggerAxis();
		double leftTriggerSpeed = this.controller.getLeftTriggerAxis();

		// joystick input, negative because y axis on XBoxController is upside down
		double joystickSpeed = -this.controller.getRawAxis(this.motorAxis);
		joystickSpeed = squareDeadzone(joystickSpeed, Constants.OperatorConstants.joystickDeadzone); // apply deadzone anyway

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
			joystickSpeed = quadratic(joystickSpeed); // apply response curve to user input
			motorSetSpeed = joystickSpeed;
		}

		// if spinning against the pawl, open the pawl servo
		if (motorSetSpeed < 0.0d) {
			servo.setPulseTimeMicroseconds(this.disengageServoPos);
			//servo.setPosition(this.disengageServoPos);
			// start a timer if we are just now pressing the button
			if (!this.servoDisengaged) {
				this.startServoTime = System.currentTimeMillis();
				this.servoDisengaged = true;
			}
		} else {
			// spinning with from pawl
			servo.setPulseTimeMicroseconds(this.engageServoPos);
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

	}
}