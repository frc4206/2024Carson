// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkConfiguration;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase implements SparkDefaultMethods {
	public CANSparkFlex pivotMotor = new CANSparkFlex(Constants.Pivot.pivotMotorID, MotorType.kBrushless);
	public RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
	public SparkPIDController pivotController = pivotMotor.getPIDController();
	SparkConfiguration pivotConfig;

	double[][] angleData = {
		{Constants.Pivot.pivotDistanceToRobotCenter+Constants.Field.subwooferLength+ Units.inchesToMeters(34), 6.476},
		{Constants.Pivot.pivotDistanceToRobotCenter+Constants.Field.subwooferLength+ Units.inchesToMeters(52), 5.538},
		{Constants.Pivot.pivotDistanceToRobotCenter+Constants.Field.subwooferLength+ Units.inchesToMeters(62.75), 5.121},
		{Constants.Pivot.pivotDistanceToRobotCenter+Constants.Field.subwooferLength+ Units.inchesToMeters(68), 4.475},
		{Constants.Pivot.pivotDistanceToRobotCenter+Constants.Field.subwooferLength+ Units.inchesToMeters(34), 6.476}
	};
	InterpolatingTreeTableSubsystem angleTree;

	public enum ShooterPositions {
		NONE,
		CLOSE,
		SPIKE,
		PODIUM,
		UNDER,
		STAGE,
		WING,
		AMPLIFIER
	}

	public ShooterPositions position = ShooterPositions.NONE;

	public PivotSubsystem() {
		angleTree = new InterpolatingTreeTableSubsystem(angleData);

		pivotConfig = new SparkConfiguration(
			true,
			false,
			pivotMotor, 
			false, 
			IdleMode.kBrake, 
			40, 
			pivotEncoder, 
			pivotController, 
			Constants.Pivot.pivotKP, 
			Constants.Pivot.pivotKI, 
			Constants.Pivot.pivotKIZone, 
			Constants.Pivot.pivotKD, 
			0, 
			Constants.Pivot.pivotMaxVel, 
			Constants.Pivot.pivotMaxAccel, 
			Constants.Pivot.pivotAllowedError
		);
		pivotMotor.setClosedLoopRampRate(0.25);
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
	
	public void autoAdjust(double distFromSpeaker){
		if (distFromSpeaker > 6.0){
			setPosition(1);
		}
		double newAngle = angleTree.getInterpolatedValue(distFromSpeaker);
		setPosition(newAngle);
	}
	
	public void changePosition(ShooterPositions newPosition){
		position = newPosition;
	}

	public void togglePivotMode(){
		if (position == ShooterPositions.NONE){
			position = ShooterPositions.PODIUM;
		} else {
			position = ShooterPositions.NONE;
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
			default:
				break;
			}
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Pivot position", pivotEncoder.getPosition());

		if(position != ShooterPositions.NONE) {
			setFieldRelativePosition();
		} else {
			// autoAdjust(GlobalVariables.distanceToSpeaker);
		}
	}
}
