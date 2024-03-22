// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class FlywheelSubsystem extends SubsystemBase {
	private CANSparkFlex topFlyMotor = new CANSparkFlex(Constants.Flywheel.topFlywheelMotorID, MotorType.kBrushless); 
	private CANSparkFlex bottomFlyMotor = new CANSparkFlex(Constants.Flywheel.bottomFlywheelMotorID, MotorType.kBrushless);
	private RelativeEncoder topFlyEncoder = topFlyMotor.getEncoder();
	private RelativeEncoder bottomFlyEncoder = bottomFlyMotor.getEncoder();
	private SparkPIDController topFlyPIDController = topFlyMotor.getPIDController();
	private SparkPIDController bottomFlyPIDController = bottomFlyMotor.getPIDController();
	SparkConfig topFlyConfig;
	SparkConfig bottomFlyConfig;

	public FlywheelSubsystem() {
		topFlyConfig = Constants.Flywheel.topFlywheelConfig;
		topFlyConfig.configureController(topFlyMotor, topFlyEncoder, topFlyPIDController);
		topFlyConfig.applyAllConfigurations();

		bottomFlyConfig = Constants.Flywheel.bottomFlywheelConfig;
		bottomFlyConfig.configureController(bottomFlyMotor, bottomFlyEncoder, bottomFlyPIDController);
		bottomFlyConfig.applyAllConfigurations();
	}

	public boolean shooterAtVelocity(double setVelocity){
		return (
			(Math.abs(topFlyEncoder.getVelocity() - setVelocity) < 50) &&
			(Math.abs(bottomFlyEncoder.getVelocity() - setVelocity) < 50)
		);
	}

	public boolean shooterAtVelocities(double topSetVelo, double bottomSetVelo){
		return (
			(Math.abs(topFlyEncoder.getVelocity() - topSetVelo) < 50) && 
			(Math.abs(bottomFlyEncoder.getVelocity() - bottomSetVelo) < 50)
		);
	}

	public void shooterToVelocity(double setVelocity) {
		topFlyPIDController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
		bottomFlyPIDController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
	}

	public void shooterToVelocity(double setVelocityUpper, double setVelocityLower) {
		topFlyPIDController.setReference(setVelocityUpper, CANSparkFlex.ControlType.kVelocity);
		bottomFlyPIDController.setReference(setVelocityLower, CANSparkFlex.ControlType.kVelocity);
	}

	public void shooterToDuty(double percent) {
		topFlyMotor.set(percent);
		bottomFlyMotor.set(percent);
	}

	public void shooterToDuty(double upperPercent, double lowerPercent){
		topFlyMotor.set(upperPercent);
		bottomFlyMotor.set(lowerPercent);
	}

	@Override
	public void periodic() {
		GlobalVariables.Flywheel.topVelo = topFlyEncoder.getVelocity();
		GlobalVariables.Flywheel.bottomVelo = bottomFlyEncoder.getVelocity();
		SmartDashboard.putNumber("topVelo", GlobalVariables.Flywheel.topVelo);
		SmartDashboard.putNumber("bottomVelo", GlobalVariables.Flywheel.bottomVelo);
	}
}