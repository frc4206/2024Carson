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
		AMPLIFIER
	}

	public ShooterPositions position = ShooterPositions.AUTO;

	public PivotSubsystem() {
		pivotConfig = Constants.Pivot.pivotConfig;
		pivotConfig.configureController(pivotMotor, pivotEncoder, pivotController);
		pivotConfig.applyConfigurations();
	}
	
	public boolean pivotWithinRange(double desiredPosition){
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
		} else {
			position = ShooterPositions.AUTO;
		}
	}

	public void togglePivotMode(){
		if (position == ShooterPositions.AUTO){
			position = ShooterPositions.CLOSE;
		} else {
			position = ShooterPositions.AUTO;
		}
	}

	public void toggleAmpMode(){
		if (position != ShooterPositions.AMPLIFIER){
			position = ShooterPositions.AMPLIFIER;
			GlobalVariables.Shooter.toAmpVelo = true;
		} else {
			position = ShooterPositions.AUTO;
			GlobalVariables.Shooter.toAmpVelo = false;
		} 
	}

	public void toggleSubwoofer(){
		if (position != ShooterPositions.SUBWOOFER){
			position = ShooterPositions.SUBWOOFER;
		} else {
			position = ShooterPositions.AUTO;
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
			case MANUAL:
				break;
			default:
				break;
			}
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Pivot position", pivotEncoder.getPosition());

		if(position != ShooterPositions.AUTO) {
			setFieldRelativePosition();
		} else {
			autoPivot();
		}
	}
}
