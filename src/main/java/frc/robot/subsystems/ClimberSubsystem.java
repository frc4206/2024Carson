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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
	private CANSparkFlex climber_motor; //= new CANSparkFlex(Constants.Climber.climberLeftFollowID, MotorType.kBrushless);
	private RelativeEncoder climber_encoder; //= climberLeftMotor.getEncoder();
	private SparkPIDController climber_PID_controller; // = climberLeftMotor.getPIDController();

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

	public void climbToPosition(double setpoint){
		climber_PID_controller.setReference(setpoint, ControlType.kPosition);
	}

	public void climbSTOP() {
		climber_motor.set(0);
	}

	public void climberUP(){
		climber_motor.set(-0.2);
	}

	public void climbDOWN(){
		climber_motor.set(0.2);
	}

	public void setPosition(double pos){
		servo.setPosition(pos);
	}

	@Override
	public void periodic() {}
}
