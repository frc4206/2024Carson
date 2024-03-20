// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.FeedbackConfig;
import frc.lib.util.spark.sparkConfig.MotorConfig;
import frc.lib.util.spark.sparkConfig.PIDConfig;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class PivotSubsystem extends SubsystemBase implements SparkDefaultMethods {
	public CANSparkFlex pivotMotor = new CANSparkFlex(Constants.Pivot.pivotMotorID, MotorType.kBrushless);
	public RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
	public SparkPIDController pivotController = pivotMotor.getPIDController();
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
		pivotConfig = new SparkConfig(
			new FeedbackConfig(-1, 1, Constants.Pivot.pivotMaxVel, Constants.Pivot.pivotMaxAccel, Constants.Pivot.pivotAllowedError), 
			new MotorConfig(Constants.Pivot.pivotMotorID, false, IdleMode.kBrake, 40, 0.25), 
			new PIDConfig(Constants.Pivot.pivotkP, Constants.Pivot.pivotkI, Constants.Pivot.pivotkIZone, Constants.Pivot.pivotkD, 0), 
			pivotMotor, 
			pivotEncoder, 
			pivotController, 
			true, 
			false
		);
	}
	
	public void resetPivot(){
		resetMotor(pivotEncoder);
	}
	
	public void runPivot(double speed) {
		setMotorSpeed(pivotMotor, speed);
	}
	
	public void setPosition(double angle) {
		motorGoToPosition(pivotController, angle);
	}

	public void autoPivot() {
		motorGoToPosition(pivotController, SmartDashboard.getNumber("DesPos", 2) < 0.5 ? 2 : SmartDashboard.getNumber("DesPos", 2));
	}
	
	public void pivotWithMove(){
		motorGoToPosition(
			pivotController, 
			SmartDashboard.getNumber("DesPos", 2) - (1/3)*Math.sqrt(
				Math.pow(SmartDashboard.getNumber("translationX", 0), 2) + 
				Math.pow(SmartDashboard.getNumber("translationY", 0), 2)
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
			GlobalVariables.toAmpVelo = true;
		} else {
			position = ShooterPositions.AUTO;
			GlobalVariables.toAmpVelo = false;
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
				setPosition(Constants.Pivot.stagePosition);
				break;
			case UNDER:
				setPosition(Constants.Pivot.underPosition);
				break;
			case PODIUM:
				setPosition(Constants.Pivot.podiumPosition); 
				break;
			case CLOSE:
				setPosition(Constants.Pivot.closePosition);
				break;
			case AMPLIFIER:
				setPosition(Constants.Pivot.ampPosition);
				break;
			case SUBWOOFER:
				setPosition(Constants.Pivot.subwooferPosition);
				break;
			case MANUAL:
				break;
			default:
				break;
			}
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Pivot position", pivotEncoder.getPosition());

		// if(position != ShooterPositions.AUTO) {
		// 	setFieldRelativePosition();
		// } else {
		// 	autoPivot();
		// }
	}
}
