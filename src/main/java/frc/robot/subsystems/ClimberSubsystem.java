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
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private CANSparkFlex climberMotor = new CANSparkFlex(Constants.Climber.climberMotorID, MotorType.kBrushless);
	private RelativeEncoder climberEncoder = climberMotor.getEncoder();
	private SparkPIDController climberPIDController = climberMotor.getPIDController();
	SparkConfig climberConfig;

	PWM servo = new PWM(Constants.Climber.servoID);

	private XboxController controller;

	private boolean servoDisengaged = false;
	private long startServoTime = 0;
	private long currentTime = 0;
	private double joystickSpeed = 0;	
	private double motorSetSpeed = 0;

	public ClimberSubsystem() {
		climberConfig = Constants.Climber.climberConfig;
		climberConfig.configureController(climberMotor, climberEncoder, climberPIDController);
		climberConfig.applyAllConfigurations();
	}

	public void setupController(XboxController controller){
		if (this.controller == null){
			this.controller = controller;
		}
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

	public void engageServo(){
		servo.setPulseTimeMicroseconds(Constants.Climber.servoEngage);
	}

	public void disengageServo(){
		servo.setPulseTimeMicroseconds(Constants.Climber.servoDisengage);
	}

	public void climberToDuty(double desiredDuty){
		motorToDuty(climberMotor, desiredDuty);
	}

	public void resetClimber(){
		resetEncoder(climberEncoder);
	}

	public void climberToPosition(double desiredPosition){
		motorToPosition(climberPIDController, desiredPosition);
	}

	@Override
	public void periodic() {
		joystickSpeed = -controller.getRawAxis(XboxController.Axis.kLeftY.value);
		joystickSpeed = squareDeadzone(joystickSpeed, Constants.OperatorConstants.joystickDeadzone);
		motorSetSpeed = 0.0d;

		if (joystickSpeed != 0.0d) {
			joystickSpeed = cubic(joystickSpeed);
			motorSetSpeed = joystickSpeed;
		}

		if (motorSetSpeed < 0.0d) {
			disengageServo();
			if (!servoDisengaged) {
				startServoTime = System.currentTimeMillis();
				servoDisengaged = true;
			}
		} else {
			engageServo();
			servoDisengaged = false;
		}

		currentTime = System.currentTimeMillis();

		if (currentTime - startServoTime <= Constants.Climber.disengageDurationMilliseconds && servoDisengaged) {
			motorSetSpeed = 0.0d;
		}

		climberToDuty(motorSetSpeed);
		SmartDashboard.putNumber("climberCurrent", climberMotor.getOutputCurrent());
	}
}