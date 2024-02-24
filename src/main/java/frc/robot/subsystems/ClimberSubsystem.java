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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
	private CANSparkFlex climberRightLead = new CANSparkFlex(Constants.Climber.climberRightLeadID, MotorType.kBrushless);
	private CANSparkFlex climberLeftFollow = new CANSparkFlex(Constants.Climber.climberLeftFollowID, MotorType.kBrushless);

	private SparkPIDController climbLeadPid;
	public RelativeEncoder climbRightLeadEncoder; /* top encoder */
	private RelativeEncoder climbLeftFollowEncoder; /* bottom encoder */
	
	PWM servoRight = new PWM(1);
	PWM servoLeft = new PWM(2);

	//private DigitalInput TopClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);
	//private DigitalInput BottomClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);

	public ClimberSubsystem() {
		climberRightLead.restoreFactoryDefaults();
		climberLeftFollow.restoreFactoryDefaults();
		climberRightLead.setIdleMode(IdleMode.kBrake);
		climberRightLead.setInverted(false);
		climberLeftFollow.setInverted(false);

		climberLeftFollow.follow(climberRightLead);

		climbRightLeadEncoder = climberRightLead.getEncoder();
		climbLeftFollowEncoder = climberLeftFollow.getEncoder();
		climbLeadPid = climberRightLead.getPIDController();
		
		climbLeadPid.setFeedbackDevice(climbRightLeadEncoder);
		climbLeadPid.setP(0.02);
		climbLeadPid.setI(9e-8);
		climbLeadPid.setD(0.0);
		climbLeadPid.setFF(0.0);
		climbLeadPid.setSmartMotionMaxVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecondfast, 0);
		climbLeadPid.setSmartMotionMinOutputVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 0);
		climbLeadPid.setSmartMotionMaxAccel(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 0);
		climbLeadPid.setSmartMotionAllowedClosedLoopError(5, 0);
	}

	public void climbSTOP() {
		climberRightLead.set(0);
		climberLeftFollow.set(0);
	}

	public void climbUPRight() { climberRightLead.set(0.2); }

	public void climbDOWNRight() { climberRightLead.set(-0.2); }

	public void climbUPLeft() { climberLeftFollow.set(0.2); }

	public void climbDOWNLeft() { climberLeftFollow.set(-0.2); }

	public void GoToSetpoint(double setpoint) { climbLeadPid.setReference(setpoint, ControlType.kPosition, 0); }

	public void setPositionRight(double pos) { servoRight.setPosition(pos); }

	public void setPositionLeft(double pos) { servoLeft.setPosition(pos); }

	public void runServoLeft(double speed) { servoLeft.setSpeed(speed); }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//if(TopClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(0);
		//}
		//if(BottomClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(Constants.Climber.climberResetPosition);
		//}
		SmartDashboard.putNumber("Climber Right Position", climbRightLeadEncoder.getPosition());
		SmartDashboard.putNumber("Climber Left Position", climbLeftFollowEncoder.getPosition());

		SmartDashboard.putNumber("servo Right", servoRight.getPosition());
		SmartDashboard.putNumber("servo Left", servoLeft.getPosition());

	}
}
