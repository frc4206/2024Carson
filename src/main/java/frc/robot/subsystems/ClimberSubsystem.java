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
	private CANSparkFlex climberRightLead = new CANSparkFlex(Constants.Climber.climberRightLeadID, MotorType.kBrushless);
	private CANSparkFlex climberLeftFollow = new CANSparkFlex(Constants.Climber.climberLeftFollowID, MotorType.kBrushless);
	private SparkPIDController climbLeadPid;
	public RelativeEncoder climbRightLeadEncoder;

	PWM servoRight = new PWM(1);
	PWM servoLeft = new PWM(2);

	public ClimberSubsystem() {
		climberRightLead.restoreFactoryDefaults();
		climberLeftFollow.restoreFactoryDefaults();
		climberRightLead.setIdleMode(IdleMode.kBrake);
		climberRightLead.setInverted(true);
		climberLeftFollow.setInverted(false);

		//climberLeftFollow.follow(climberRightLead);

		climbRightLeadEncoder = climberRightLead.getEncoder();
		climbLeadPid = climberRightLead.getPIDController();
		
		climbLeadPid.setFeedbackDevice(climbRightLeadEncoder);
		climbLeadPid.setP(Constants.Climber.climberkP);
		climbLeadPid.setI(Constants.Climber.climberkI);
		climbLeadPid.setIZone(Constants.Climber.climberkIZone);
		climbLeadPid.setD(Constants.Climber.climberkD);
		climbLeadPid.setSmartMotionMaxVelocity(Constants.Climber.climberMaxVelo, 0);
		climbLeadPid.setSmartMotionMinOutputVelocity(-Constants.Climber.climberMaxVelo, 0);
		climbLeadPid.setSmartMotionMaxAccel(Constants.Climber.climberMaxAcc, 0);
		climbLeadPid.setSmartMotionAllowedClosedLoopError(Constants.Climber.climberAllowedError, 0);
  	}

	public void climbSTOP() {
		climberRightLead.set(0);
		climberLeftFollow.set(0);
	}

	public void climberUP(){
		climberRightLead.set(0.2);
		climberLeftFollow.set(0.2);
	}

	public void climbUPRight() { 
		climberRightLead.set(0.2);
	}

	public void climbUPLeft() { 
		climberLeftFollow.set(0.2); 
	}

	public void climbDOWN(){
		climberRightLead.set(-0.2);
		climberLeftFollow.set(-0.2);
	}

	public void climbDOWNRight() { 
		climberRightLead.set(-0.2); 
	}

	public void climbDOWNLeft() { 
		climberLeftFollow.set(-0.2); 
	}

	public void GoToSetpoint(double setpoint) { 
		climbLeadPid.setReference(setpoint, ControlType.kPosition, 0); 
	}

	public void setPosition(double pos){
		servoLeft.setPosition(pos);
		servoRight.setPosition(pos);
	}

	public void setPosition(double leftPos, double rightPos){
		servoLeft.setPosition(leftPos);
		servoRight.setPosition(rightPos);
	}

	public void setPositionRight(double pos) { 
		servoRight.setPosition(pos); 
	}

	public void setPositionLeft(double pos) { 
		servoLeft.setPosition(pos); 
	}

	@Override
	public void periodic() {
		//if(TopClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(0);
		//}
		//if(BottomClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(Constants.Climber.climberResetPosition);
		//}

		// SmartDashboard.putNumber("Climber Right Position", climbRightLeadEncoder.getPosition());
		// SmartDashboard.putNumber("Climber Left Position", climbLeftFollowEncoder.getPosition());

		// SmartDashboard.putNumber("servo Right", servoRight.getPosition());
		// SmartDashboard.putNumber("servo Left", servoLeft.getPosition());
	}
}
