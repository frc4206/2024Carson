// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

	/* Variables */
	private CANSparkFlex climberLeaderMotor = new CANSparkFlex(Constants.Climber.climberLeaderMotorID, MotorType.kBrushless);
	//private CANSparkFlex climberFollowerMotor = new CANSparkFlex(Constants.Climber.climberFollowerID, MotorType.kBrushless);
	//private CANSparkFlex climberLeadMotor = new CANSparkFlex(Constants.Climber.climberLeaderMotorID, MotorType.kBrushless);
	private CANSparkFlex climberFollowerMotor = new CANSparkFlex(Constants.Climber.climberFollowerID, MotorType.kBrushless);

	private SparkPIDController climbLeadPid;
	private RelativeEncoder climbLeadEncoder; /* top encoder */
	//private RelativeEncoder climbBottomEncoder; /* bottom encoder */
	PWM servo1 = new PWM(Constants.Climber.SERVO_ONE_CHANNEL);
	PWM servo2 = new PWM(Constants.Climber.SERVO_TWO_CHANNEL);

	//private DigitalInput TopClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);
	//private DigitalInput BottomClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);

	/* Method Constructor */
	public ClimberSubsystem() {
		climberLeaderMotor.restoreFactoryDefaults();
		climberFollowerMotor.restoreFactoryDefaults();
		climberLeaderMotor.setInverted(false);
		climberFollowerMotor.setInverted(false);

		climberFollowerMotor.follow(climberLeaderMotor);

		climbLeadEncoder = climberLeaderMotor.getEncoder();
		climbLeadPid = climberLeaderMotor.getPIDController();
		
		climbLeadPid.setFeedbackDevice(climbLeadEncoder);
		climbLeadPid.setP(Constants.Climber.vortexClimberLeadKP);
		climbLeadPid.setI(Constants.Climber.vortexClimberLeadKI);
		climbLeadPid.setD(Constants.Climber.vortexClimberLeadKD);
		//climbLeadPid.setFF(Constants.Climber.vortexClimberLeadFF);
		//climbLeadPid.setSmartMotionMaxVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecondfast, 0);
		//climbLeadPid.setSmartMotionMinOutputVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 0);
		//climbLeadPid.setSmartMotionMaxAccel(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 0);
		//climbLeadPid.setSmartMotionAllowedClosedLoopError(5, 0);

		
	}

	public void climbSTOP() {
		climberLeaderMotor.set(0);
		//climberFollowerMotor.set(0);
	}

	public void climbUP() {
		climberLeaderMotor.set(0.8);
		//climberFollowerMotor.set(0.8);
	}

	public void climbDOWN() {
		climberLeaderMotor.set(-0.8);
		//climberFollowerMotor.set(-0.8);
	}

	public void GoToSetpoint(double setpoint) {
		climbLeadPid.setReference(setpoint, ControlType.kPosition, 0);
	}

	public void setPosition(double pos) {
		servo1.setPosition(pos);
		servo2.setPosition(pos);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//if(TopClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(0);
		//}
		//if(BottomClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(Constants.Climber.climberResetPosition);
		//} 
	}
}
