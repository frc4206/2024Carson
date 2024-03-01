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
	private CANSparkFlex climberLeftFollow = new CANSparkFlex(Constants.Climber.climberLeftFollowID, MotorType.kBrushless);
	// private SparkPIDController climbLeadPid;

	PWM servoLeft = new PWM(2);

	public ClimbLeftSubsystem() {
		climberLeftFollow.restoreFactoryDefaults();
		climberLeftFollow.setIdleMode(IdleMode.kBrake);
		climberLeftFollow.setInverted(false);
		climberLeftFollow.burnFlash();
  	}

	public void climbSTOP() {
		climberLeftFollow.set(0);
	}

	public void climberUP(){
		climberLeftFollow.set(-0.2);
	}

	public void climbDOWN(){
		climberLeftFollow.set(0.2);
	}

	// public void GoToSetpoint(double setpoint) { 
	// 	climbLeadPid.setReference(setpoint, ControlType.kPosition, 0); 
	// }

	public void setPosition(double pos){
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
