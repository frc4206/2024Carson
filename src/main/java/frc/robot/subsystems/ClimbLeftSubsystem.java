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

public class ClimbLeftSubsystem extends SubsystemBase {
	private CANSparkFlex climberLeftMotor = new CANSparkFlex(Constants.Climber.climberLeftFollowID, MotorType.kBrushless);
	private RelativeEncoder climberLeftEncoder = climberLeftMotor.getEncoder();
	private SparkPIDController climberLeftPIDController = climberLeftMotor.getPIDController();

	PWM servoLeft = new PWM(Constants.Climber.servoLeftID);

	public ClimbLeftSubsystem() {
		climberLeftMotor.restoreFactoryDefaults();
		climberLeftMotor.setIdleMode(IdleMode.kBrake);
		climberLeftMotor.setInverted(false);
		climberLeftMotor.setSmartCurrentLimit(40);
		climberLeftMotor.burnFlash();
		
		climberLeftEncoder.setPosition(0);
		climberLeftPIDController.setFeedbackDevice(climberLeftEncoder);

		climberLeftPIDController.setP(Constants.Climber.climberkP);
		climberLeftPIDController.setI(Constants.Climber.climberkI);
		climberLeftPIDController.setIZone(Constants.Climber.climberkIZone);
		climberLeftPIDController.setD(Constants.Climber.climberkD);
		climberLeftPIDController.setOutputRange(-1, 1, 0);
		climberLeftPIDController.setSmartMotionMaxVelocity(Constants.Climber.climberMaxVelo, 0);
		climberLeftPIDController.setSmartMotionMaxAccel(Constants.Climber.climberMaxAcc, 0);
		climberLeftPIDController.setSmartMotionAllowedClosedLoopError(Constants.Climber.climberAllowedError, 0);
  	}

	public void climbToPosition(double setpoint){
		climberLeftPIDController.setReference(setpoint, ControlType.kPosition);
	}

	public void climbSTOP() {
		climberLeftMotor.set(0);
	}

	public void climberUP(){
		climberLeftMotor.set(-0.2);
	}

	public void climbDOWN(){
		climberLeftMotor.set(0.2);
	}

	public void setPosition(double pos){
		servoLeft.setPosition(pos);
	}

	@Override
	public void periodic() {}
}
