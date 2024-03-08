// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkConfiguration;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private CANSparkFlex climberMotor;
	private RelativeEncoder climberEncoder;
	private SparkPIDController climberPIDController;
	private PWM servo;
	SparkConfiguration climberConfig;

	public ClimberSubsystem(int canID, boolean motorisInverted, int currentLimit, int servoID) {
        climberMotor = new CANSparkFlex(canID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        climberPIDController = climberMotor.getPIDController();
		servo = new PWM(servoID);

		climberConfig = new SparkConfiguration(
			true,
			false,
			climberMotor, 
			motorisInverted, 
			IdleMode.kBrake, 
			currentLimit, 
			climberEncoder, 
			climberPIDController, 
			Constants.Climber.climberkP, 
			Constants.Climber.climberkI, 
			Constants.Climber.climberkIZone, 
			Constants.Climber.climberkD, 
			0, 
			Constants.Climber.climberMaxVelo, 
			Constants.Climber.climberMaxAcc, 
			Constants.Climber.climberAllowedError
		);
  	}

	public void climbToPosition(double setpoint){
		motorGoToPosition(climberPIDController, setpoint);
	}

	public void climbToDuty(double setDuty){
		setMotorSpeed(climberMotor, setDuty);
	}

	public void setPosition(double pos){
		servo.setPosition(pos);
	}

	@Override
	public void periodic() {}
}
