// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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

public class PivotSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private CANSparkFlex pivotMotor;
	private RelativeEncoder pivotEncoder;
	private SparkPIDController pivotController;
	SparkConfig pivotConfig;
	ControllerConfig controllerConfig;

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
		controllerConfig = new ControllerConfig(Constants.Pivot.pivotMotorID, pivotMotor, pivotEncoder, pivotController);
		pivotMotor = controllerConfig.getMotor();
		pivotEncoder = controllerConfig.getEncoder();
		pivotController = controllerConfig.getPIDController();

		pivotConfig = new SparkConfig(
			new FeedbackConfig(Constants.Feedback.defaultMinDuty, Constants.Feedback.defaultMaxDuty, Constants.Pivot.pivotMaxVel, Constants.Pivot.pivotMaxAccel, Constants.Pivot.pivotAllowedError), 
			new MotorConfig(Constants.Pivot.pivotMotorID, Constants.Pivot.pivotIsInverted, Constants.Pivot.idleMode, Constants.Pivot.pivotCurrentLimit, Constants.Pivot.pivotClosedLoopRampRate), 
			new PIDConfig(Constants.Pivot.pivotkP, Constants.Pivot.pivotkI, Constants.Pivot.pivotkIZone, Constants.Pivot.pivotkD, Constants.Pivot.pivotkFF), 
			pivotMotor, 
			pivotEncoder, 
			pivotController, 
			Constants.Pivot.shouldRestore, 
			Constants.Pivot.shouldBurn
		);
		pivotConfig.applyConfig();
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
