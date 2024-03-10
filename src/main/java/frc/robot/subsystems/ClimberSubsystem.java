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
	private CANSparkFlex climber_motor;
	private RelativeEncoder climber_encoder;
	private SparkPIDController climber_PID_controller;

	private XboxController controller;
	private int motor_axis;

	public double engageServoPos;
	public double disengageServoPos;
	private boolean servoDisengaged = false;
	private long startServoTime = 0;
	private long disengageDuractionMilliseconds = 200; // 0.2 seconds (human reaction time)

	private final double default_motor_speed = 0.1d;
	private final double DEADZONE = 0.1;

	PWM servo;// = new PWM(Constants.Climber.servoLeftID);

	public ClimberSubsystem(int motor_CAN_id, boolean invert_motor, int current_limit, int servo_id) {
		climber_motor = new CANSparkFlex(motor_CAN_id, MotorType.kBrushless);
		climber_encoder = climber_motor.getEncoder();
		climber_PID_controller = climber_motor.getPIDController();

		servo = new PWM(servo_id);

		climber_motor.restoreFactoryDefaults();
		climber_motor.setIdleMode(IdleMode.kBrake);
		climber_motor.setInverted(invert_motor);
		climber_motor.setSmartCurrentLimit(current_limit);
		climber_motor.burnFlash();

		climber_encoder.setPosition(0);
		climber_PID_controller.setFeedbackDevice(climber_encoder);

		climber_PID_controller.setP(Constants.Climber.climberkP);
		climber_PID_controller.setI(Constants.Climber.climberkI);
		climber_PID_controller.setIZone(Constants.Climber.climberkIZone);
		climber_PID_controller.setD(Constants.Climber.climberkD);
		climber_PID_controller.setOutputRange(-1, 1, 0);
		climber_PID_controller.setSmartMotionMaxVelocity(Constants.Climber.climberMaxVelo, 0);
		climber_PID_controller.setSmartMotionMaxAccel(Constants.Climber.climberMaxAcc, 0);
		climber_PID_controller.setSmartMotionAllowedClosedLoopError(Constants.Climber.climberAllowedError, 0);
	}

	public boolean setupController(XboxController controller, int axis) {
		if (controller == null)
			return false;
		this.controller = controller;
		this.motor_axis = axis;
		return true;
	}

	public void climbToPosition(double setpoint) {
		climber_PID_controller.setReference(setpoint, ControlType.kPosition);
	}

	public void climbSTOP() {
		climber_motor.set(0);
	}

	public void climberUP() {
		climber_motor.set(default_motor_speed);
	}

	public void climbDOWN() {
		climber_motor.set(-default_motor_speed);
	}

	public void setPosition(double pos) {
		servo.setPosition(pos);
	}

	public double square_deadzone(double val, double deadzone) {
		double dead_zoned = (Math.abs(val) >= deadzone ? map(Math.abs(val), deadzone, 1.0d, 0.0d, 1.0d) : 0.0d);
		return val >= 0.0d ? dead_zoned : -dead_zoned;
	}

	public double quadratic(double val) {
		return val >= 0.0d ? (val * val) : -(val * val);
	}

	public double map(double val, double in_min, double in_max, double out_min, double out_max) {
		return ((val - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
	}

	@Override
	public void periodic() {
		double motor_speed_set = 0.0d;

		// trigger input
		double rght_trigger_speed = this.controller.getRightTriggerAxis();
		double left_trigger_speed = this.controller.getLeftTriggerAxis();

		// joystick input, negative because y axis on XBoxController is upside down
		double jystck_speed = -this.controller.getRawAxis(this.motor_axis);
		jystck_speed = square_deadzone(jystck_speed, this.DEADZONE); // apply deadzone anyway

		// ignore if both right and left trigger are pressed
		if (rght_trigger_speed > 0.0d && left_trigger_speed > 0.0d) {
			climber_motor.set(0.0d); // safe to zero for safety
			return; // return early, multiple inputs are unclear so ignore
		}

		// by this point, either one is these is NOT zero or both are
		if (left_trigger_speed > 0.0d) {
			motor_speed_set = -left_trigger_speed;
		}
		if (rght_trigger_speed > 0.0d) {
			motor_speed_set = rght_trigger_speed; // opposite direction
		}

		// if nothing set, safe to use joystick input
		if (motor_speed_set == 0.0d && jystck_speed != 0.0d) {
			jystck_speed = quadratic(jystck_speed); // apply response curve to user input
			motor_speed_set = jystck_speed;
		}

		// if spinning against the pawl, open the pawl servo
		if (motor_speed_set < 0.0d) {
			servo.setPosition(this.disengageServoPos);
			// start a timer if we are just now pressing the button
			if (!this.servoDisengaged) {
				this.startServoTime = System.currentTimeMillis();
				this.servoDisengaged = true;
			}
		} else {
			// spinning with from pawl
			servo.setPosition(this.engageServoPos);
			this.servoDisengaged = false;
		}

		long currentTime = System.currentTimeMillis();

		// if not enough time has passed for the servos to disengage
		// then we should not start moving the motors yet
		if (currentTime - startServoTime <= this.disengageDuractionMilliseconds && this.servoDisengaged) {
			motor_speed_set = 0.0d;
		}

		// set motor
		climber_motor.set(motor_speed_set);

	}
}
