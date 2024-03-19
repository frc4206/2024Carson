// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.sparkConfig.FeedbackConfig;
import frc.lib.util.spark.sparkConfig.MotorConfig;
import frc.lib.util.spark.sparkConfig.PIDConfig;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants; 
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

public class FlywheelSubsystem extends SubsystemBase {
	private static CANSparkFlex upperFlyMotor; 
	private static CANSparkFlex lowerFlyMotor;
	private static RelativeEncoder upperFlyEncoder;
	private static RelativeEncoder lowerFlyEncoder;
	private SparkPIDController upperFlyPIDController;
	private SparkPIDController lowerFlyPIDController;
	SparkConfig upperConfig;
	SparkConfig lowerConfig;

	public FlywheelSubsystem() {
		upperConfig = new SparkConfig(
			new FeedbackConfig(-1, 1, Constants.Shooter.topFlyWheelMaxVel, Constants.Shooter.topFlyWheelMaxAccel, Constants.Shooter.topFlyWheelAllowedError), 
			new MotorConfig(Constants.Shooter.shooterLeadMotorID, false, IdleMode.kCoast), 
			new PIDConfig(Constants.Shooter.topFlyWheelKP, Constants.Shooter.topFlyWheelKI, Constants.Shooter.topFlyWheelKIZone, Constants.Shooter.topFlyWheelKD, 0), 
			upperFlyMotor, 
			upperFlyEncoder, 
			upperFlyPIDController, 
			false, 
			true
		);

		lowerConfig = new SparkConfig(
			new FeedbackConfig(-1, 1, Constants.Shooter.bottomFlyWheelMaxVel, Constants.Shooter.bottomFlyWheelMaxAccel, Constants.Shooter.bottomFlyWheelAllowedError), 
			new MotorConfig(Constants.Shooter.shooterFollowerID, true, IdleMode.kCoast), 
			new PIDConfig(Constants.Shooter.bottomFlyWheelKP, Constants.Shooter.bottomFlyWheelKI, Constants.Shooter.bottomFlyWheelKIZone, Constants.Shooter.bottomFlyWheelKD, 0), 
			lowerFlyMotor, 
			lowerFlyEncoder, 
			lowerFlyPIDController, 
			false, 
			true
		);
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

	public void setVelocity(double setVelocity) {
		upperFlyPIDController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
		lowerFlyPIDController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
	}

	public void setVelocity(double setVelocityUpper, double setVelocityLower) {
		upperFlyPIDController.setReference(setVelocityUpper, CANSparkFlex.ControlType.kVelocity);
		lowerFlyPIDController.setReference(setVelocityLower, CANSparkFlex.ControlType.kVelocity);
	}

	public void percentShooter(double percent) {
		upperFlyMotor.set(percent);
		lowerFlyMotor.set(percent);
	}

	public void percentShooter2(double upperPercent, double lowerPercent){
		upperFlyMotor.set(upperPercent);
		lowerFlyMotor.set(lowerPercent);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("topVelo", upperFlyEncoder.getVelocity());
		SmartDashboard.putNumber("bottomVelo", lowerFlyEncoder.getVelocity());
		SmartDashboard.putNumber("topcurr", upperFlyMotor.getOutputCurrent());
		SmartDashboard.putNumber("bottomcurr", lowerFlyMotor.getOutputCurrent());

		// setVelocity(SmartDashboard.getNumber("top velo", 0), SmartDashboard.getNumber("bottom velo", 0));
	}
}