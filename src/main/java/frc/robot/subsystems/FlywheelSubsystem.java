// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class FlywheelSubsystem extends SubsystemBase {
	private static CANSparkFlex upperFlyMotor = new CANSparkFlex(Constants.Flywheel.topFlywheelMotorID, MotorType.kBrushless); 
	private static CANSparkFlex lowerFlyMotor = new CANSparkFlex(Constants.Flywheel.bottomFlywheelMotorID, MotorType.kBrushless);
	private static RelativeEncoder upperFlyEncoder = upperFlyMotor.getEncoder();
	private static RelativeEncoder lowerFlyEncoder = lowerFlyMotor.getEncoder();
	private SparkPIDController upperFlyPIDController = upperFlyMotor.getPIDController();
	private SparkPIDController lowerFlyPIDController = lowerFlyMotor.getPIDController();

	public FlywheelSubsystem() {
		upperFlyMotor.setInverted(Constants.Flywheel.topIsInverted);
		lowerFlyMotor.setInverted(Constants.Flywheel.bottomIsInverted);
		upperFlyMotor.setIdleMode(Constants.Flywheel.idleMode);
		lowerFlyMotor.setIdleMode(Constants.Flywheel.idleMode);

		upperFlyPIDController.setFeedbackDevice(upperFlyEncoder);
		upperFlyPIDController.setP(Constants.Flywheel.topFlywheelkP);
		upperFlyPIDController.setI(Constants.Flywheel.topFlywheelkI);
		upperFlyPIDController.setIZone(0);
		upperFlyPIDController.setD(Constants.Flywheel.topFlywheelkD);
		upperFlyPIDController.setOutputRange(Constants.Feedback.defaultMinDuty, Constants.Feedback.defaultMaxDuty, 0);
		upperFlyPIDController.setSmartMotionMaxVelocity(Constants.Flywheel.topFlywheelMaxVelo, 0);
		upperFlyPIDController.setSmartMotionMaxAccel(Constants.Flywheel.topFlywheelMaxAcc, 0);
		upperFlyPIDController.setSmartMotionAllowedClosedLoopError(Constants.Flywheel.topFlywheelMaxError, 0);

		lowerFlyPIDController.setFeedbackDevice(lowerFlyEncoder);
		lowerFlyPIDController.setP(Constants.Flywheel.bottomFlywheelkP);
		lowerFlyPIDController.setI(Constants.Flywheel.bottomFlywheelkI);
		lowerFlyPIDController.setIZone(0);
		lowerFlyPIDController.setD(Constants.Flywheel.bottomFlywheelkD);
		lowerFlyPIDController.setOutputRange(Constants.Feedback.defaultMinDuty, Constants.Feedback.defaultMaxDuty, 0);
		lowerFlyPIDController.setSmartMotionMaxVelocity(Constants.Flywheel.bottomFlywheelMaxVelo, 0);
		lowerFlyPIDController.setSmartMotionMaxAccel(Constants.Flywheel.bottomFlywheelMaxAcc, 0);
		lowerFlyPIDController.setSmartMotionAllowedClosedLoopError(Constants.Flywheel.bottomFlywheelMaxError, 0);

		upperFlyMotor.burnFlash();
		lowerFlyMotor.burnFlash();
	}


	public static boolean shooterAtVelocity(double setVelocity){
		return (
			(Math.abs(upperFlyEncoder.getVelocity() - setVelocity) < 50) &&
			(Math.abs(lowerFlyEncoder.getVelocity() - setVelocity) < 50)
		);
	}

	public static boolean shooterAtVelocities(double topSetVelo, double bottomSetVelo){
		return (
			(Math.abs(upperFlyEncoder.getVelocity() - topSetVelo) < 50) && 
			(Math.abs(lowerFlyEncoder.getVelocity() - bottomSetVelo) < 50)
		);
	}

	public void shooterToVelocity(double setVelocity) {
		upperFlyPIDController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
		lowerFlyPIDController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
	}

	public void shooterToVelocity(double setVelocityUpper, double setVelocityLower) {
		upperFlyPIDController.setReference(setVelocityUpper, CANSparkFlex.ControlType.kVelocity);
		lowerFlyPIDController.setReference(setVelocityLower, CANSparkFlex.ControlType.kVelocity);
	}

	public void shooterToDuty(double percent) {
		upperFlyMotor.set(percent);
		lowerFlyMotor.set(percent);
	}

	public void shooterToDuty(double upperPercent, double lowerPercent){
		upperFlyMotor.set(upperPercent);
		lowerFlyMotor.set(lowerPercent);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("topVelo", upperFlyEncoder.getVelocity());
		SmartDashboard.putNumber("bottomVelo", lowerFlyEncoder.getVelocity());
		SmartDashboard.putNumber("topcurr", upperFlyMotor.getOutputCurrent());
		SmartDashboard.putNumber("bottomcurr", lowerFlyMotor.getOutputCurrent());
	}
}