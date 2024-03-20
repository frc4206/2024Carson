// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.ControllerConfig;
import frc.lib.util.spark.sparkConfig.FeedbackConfig;
import frc.lib.util.spark.sparkConfig.MotorConfig;
import frc.lib.util.spark.sparkConfig.PIDConfig;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

public class FlywheelSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private static CANSparkFlex upperFlyMotor; 
	private static CANSparkFlex lowerFlyMotor;
	private static RelativeEncoder upperFlyEncoder;
	private static RelativeEncoder lowerFlyEncoder;
	private SparkPIDController upperFlyPIDController;
	private SparkPIDController lowerFlyPIDController;
	SparkConfig upperConfig;
	SparkConfig lowerConfig;
	ControllerConfig upperControllerConfig;
	ControllerConfig lowerControllerConfig;

	public FlywheelSubsystem() {
		upperControllerConfig = new ControllerConfig(Constants.Shooter.shooterLeadMotorID, upperFlyMotor, upperFlyEncoder, upperFlyPIDController);
		upperFlyMotor = upperControllerConfig.getMotor();
		upperFlyEncoder = upperControllerConfig.getEncoder();
		upperFlyPIDController = upperControllerConfig.getPIDController();

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
		upperConfig.applyConfig();

		lowerControllerConfig = new ControllerConfig(Constants.Shooter.shooterFollowerID, lowerFlyMotor, lowerFlyEncoder, lowerFlyPIDController);
		lowerFlyMotor = lowerControllerConfig.getMotor();
		lowerFlyEncoder = lowerControllerConfig.getEncoder();
		lowerFlyPIDController = lowerControllerConfig.getPIDController();

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
		lowerConfig.applyConfig();
	}


	public static boolean shooterAtVelocity(double setVelocity){
		return (
			(Math.abs(upperFlyEncoder.getVelocity() - setVelocity) < 50) &&
			(Math.abs(lowerFlyEncoder.getVelocity() - setVelocity) < 50)
		);
	}

	public static boolean shooterAtVelocity(double setVelocity, double allowableError){
		return (
			(Math.abs(upperFlyEncoder.getVelocity() - setVelocity) < allowableError) &&
			(Math.abs(lowerFlyEncoder.getVelocity() - setVelocity) < allowableError)
		);
	}
	
	public static boolean shooterAtVelocities(double topSetVelo, double bottomSetVelo){
		return (
			(Math.abs(upperFlyEncoder.getVelocity() - topSetVelo) < 50) && 
			(Math.abs(lowerFlyEncoder.getVelocity() - bottomSetVelo) < 50)
		);
	}

	public static boolean shooterAtVelocities(double topSetVelo, double bottomSetVelo, double allowableError){
		return (
			(Math.abs(upperFlyEncoder.getVelocity() - topSetVelo) < allowableError) && 
			(Math.abs(lowerFlyEncoder.getVelocity() - bottomSetVelo) < allowableError)
		);
	}

	public void setVelocity(double setVelocity) {
		motorToVelocity(upperFlyPIDController, setVelocity);
		motorToVelocity(lowerFlyPIDController, setVelocity);
	}

	public void setVelocity(double setVelocityUpper, double setVelocityLower) {
		motorToVelocity(upperFlyPIDController, setVelocityUpper);
		motorToVelocity(lowerFlyPIDController, setVelocityLower);
	}

	public void shooterToDuty(double percent) {
		motorToDuty(upperFlyMotor, percent);
		motorToDuty(lowerFlyMotor, percent);
	} 

	public void shooterToDuty(double upperPercent, double lowerPercent){
		motorToDuty(upperFlyMotor, upperPercent);
		motorToDuty(lowerFlyMotor, lowerPercent);
	}

	@Override
	public void periodic() {
		GlobalVariables.Shooter.topVelo = upperFlyEncoder.getVelocity();
		GlobalVariables.Shooter.bottomVelo = lowerFlyEncoder.getVelocity();
		
		SmartDashboard.putNumber("topVelo", GlobalVariables.Shooter.topVelo);
		SmartDashboard.putNumber("bottomVelo", GlobalVariables.Shooter.bottomVelo);
		SmartDashboard.putNumber("topcurr", upperFlyMotor.getOutputCurrent());
		SmartDashboard.putNumber("bottomcurr", lowerFlyMotor.getOutputCurrent());
	}
}