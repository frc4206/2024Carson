// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class PivotSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private CANSparkFlex pivotMotor = new CANSparkFlex(Constants.Pivot.pivotMotorID, MotorType.kBrushless);
	private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
	private SparkPIDController pivotController = pivotMotor.getPIDController();
	SparkConfig pivotConfig;

	public enum ShooterPositions {
		AUTO,
		MANUAL,
		SUBWOOFER,
		CLOSE,
		SPIKE,
		PODIUM,
		UNDER,
		STAGE,
		WING,
		AMPLIFIER,
		PASS
	}

	public ShooterPositions position = ShooterPositions.AUTO;

	public PivotSubsystem() {
		pivotConfig = Constants.Pivot.pivotConfig;
		pivotConfig.configureController(pivotMotor, pivotEncoder, pivotController);
		pivotConfig.applyAllConfigurations();
	}
	
	public boolean pivotAtPosition(double desiredPosition){
		return Math.abs(pivotEncoder.getPosition() - desiredPosition) < Constants.Pivot.pivotAllowedError;
	}

	public void resetPivotEncoder(){
		resetEncoder(pivotEncoder);
	}
	
	public void pivotToDuty(double speed) {
		motorToDuty(pivotMotor, speed);
	}
	
	public void pivotToPosition(double angle) {
		motorToPosition(pivotController, angle);
	}

	public void autoPivot() {
		pivotToPosition(GlobalVariables.Pivot.desiredPosition < Constants.Pivot.allowableThreshold ? Constants.Pivot.defaultPosition : GlobalVariables.Pivot.desiredPosition);
	}
	
	public void pivotWithMove(){
		pivotToPosition(
			GlobalVariables.Pivot.desiredPosition - (1/3)*Math.sqrt(
				Math.pow(GlobalVariables.Swerve.translationX, 2) + 
				Math.pow(GlobalVariables.Swerve.translationY, 2)
			)
		);
	}

	public void changePosition(ShooterPositions newPosition){
		position = newPosition;
	}

	public void toggleManual(){
		if (position != ShooterPositions.MANUAL){
			position = ShooterPositions.MANUAL;
			GlobalVariables.Flywheel.toAmpVelo = false;
			GlobalVariables.Flywheel.toPassVelo = false;
		} else {
			position = ShooterPositions.AUTO;
			GlobalVariables.Flywheel.toAmpVelo = false;
			GlobalVariables.Flywheel.toPassVelo = false;
		}
	}

	public void toggleAutoMode(){
		if (position == ShooterPositions.AUTO){
			position = ShooterPositions.SUBWOOFER;
			GlobalVariables.Flywheel.toAmpVelo = false;
			GlobalVariables.Flywheel.toPassVelo = false;
		} else {
			position = ShooterPositions.AUTO;
			GlobalVariables.Flywheel.toAmpVelo = false;
			GlobalVariables.Flywheel.toPassVelo = false;
		}
	}

	public void togglePassMode(){
		if (position != ShooterPositions.PASS){
			position = ShooterPositions.PASS;
			GlobalVariables.Flywheel.toAmpVelo = false;
			GlobalVariables.Flywheel.toPassVelo = true;
		} else {
			position = ShooterPositions.AUTO;
			GlobalVariables.Flywheel.toAmpVelo = false;
			GlobalVariables.Flywheel.toPassVelo = false;
		}
	}

	public void toggleAmpMode(){
		if (position != ShooterPositions.AMPLIFIER){
			position = ShooterPositions.AMPLIFIER;
			GlobalVariables.Flywheel.toAmpVelo = true;
			GlobalVariables.Flywheel.toPassVelo = false;
		} else {
			position = ShooterPositions.AUTO;
			GlobalVariables.Flywheel.toAmpVelo = false;
			GlobalVariables.Flywheel.toPassVelo = false;
		} 
	}

	public void setFieldRelativePosition() {
		switch (position){
			case STAGE:
				pivotToPosition(Constants.Pivot.stagePosition);
				break;
			case UNDER:
				pivotToPosition(Constants.Pivot.underPosition);
				break;
			case PODIUM:
				pivotToPosition(Constants.Pivot.podiumPosition); 
				break;
			case CLOSE:
				pivotToPosition(Constants.Pivot.closePosition);
				break;
			case AMPLIFIER:
				pivotToPosition(Constants.Pivot.ampPosition);
				break;
			case SUBWOOFER:
				pivotToPosition(Constants.Pivot.subwooferPosition);
				break;
			case PASS:
				pivotToPosition(Constants.Pivot.passPosition);
				break;
			case MANUAL:
				break;
			default:
				break;
			}
	}

	@Override
	public void periodic() {
		GlobalVariables.Pivot.pivotPosition = pivotEncoder.getPosition();
		SmartDashboard.putNumber("Pivot position", GlobalVariables.Pivot.pivotPosition);

		GlobalVariables.Position.distanceToSpeaker = Math.sqrt(((Limelight.limelightshooter.aprilTagResult[0]) * (Limelight.limelightshooter.aprilTagResult[0]))    +      ((Limelight.limelightshooter.aprilTagResult[2]) * (Limelight.limelightshooter.aprilTagResult[2])));
        GlobalVariables.Pivot.desiredPosition = 14.8*Math.pow(.76, GlobalVariables.Position.distanceToSpeaker);

		if(position != ShooterPositions.AUTO) {
			setFieldRelativePosition();
		} else {
			autoPivot();
		}
	}
}
