// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
  public CANSparkFlex pivotMotor = new CANSparkFlex(Constants.Shooter.shooterPivotID, MotorType.kBrushless);
  public RelativeEncoder pivotEncoder;
  public SparkPIDController pivotController;

	double[][] angleData = {{4.65, 1.03}, {4.04, 1.86}, {3.89, 1.69}, {3.12, 2.77}, {2.90, 3.21}, {2.49, 4.07}, {2.515, 4.85}};
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

		pivotEncoder = pivotMotor.getEncoder();
		
		pivotMotor.restoreFactoryDefaults();
		pivotMotor.setIdleMode(IdleMode.kBrake);
		pivotMotor.setClosedLoopRampRate(0.25);
		pivotMotor.setSmartCurrentLimit(40);

		pivotController = pivotMotor.getPIDController();
		pivotController.setFeedbackDevice(pivotEncoder);
		pivotController.setP(Constants.Pivot.pivotKP);
		pivotController.setI(Constants.Pivot.pivotKI);
		pivotController.setIZone(Constants.Pivot.pivotKIZone);
		pivotController.setD(Constants.Pivot.pivotKD);
		pivotController.setSmartMotionMaxVelocity(Constants.Pivot.pivotMaxVel, 0);
		pivotController.setSmartMotionMinOutputVelocity(Constants.Pivot.pivotMinVel, 0);
		pivotController.setSmartMotionMaxAccel(Constants.Pivot.pivotMaxAccel, 0);
		pivotController.setSmartMotionAllowedClosedLoopError(Constants.Pivot.pivotAllowedError, 0);
	}
	
	public void resetPivot(){
		pivotEncoder.setPosition(0);
	}
	
	public void runPivot(double speed) {
		pivotMotor.set(speed);
	}
	
	public void setPosition(double angle) {
		pivotController.setReference(angle, CANSparkFlex.ControlType.kPosition);
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

	public void cycleRelativePosition(){
		switch (position) {
			// case WING:
			// 	position = ShooterPositions.STAGE;
			// 	break;
			// case STAGE:
			// 	position = ShooterPositions.UNDER;
			// 	break;
			case UNDER:
				position = ShooterPositions.PODIUM;
				break;
			case PODIUM:
				position = ShooterPositions.SPIKE;
				break;
			case SPIKE:
				position = ShooterPositions.CLOSE;
				break;
			case CLOSE:
				position = ShooterPositions.WING;
				break;
			default:
				position = ShooterPositions.UNDER;
				break;
			}
	}

	public void setFieldRelativePosition() {
		switch (position){
			// case WING:
			// 	pivotController.setReference(Constants.Pivot.wingPosition, CANSparkFlex.ControlType.kPosition); 
			// 	break;
			// case STAGE:
			// 	pivotController.setReference(Constants.Pivot.stagePosition, CANSparkFlex.ControlType.kPosition); 
			// 	break;
			case UNDER:
				pivotController.setReference(Constants.Pivot.underPosition, CANSparkFlex.ControlType.kPosition); 
				break;
			case PODIUM:
				pivotController.setReference(Constants.Pivot.podiumPosition, CANSparkFlex.ControlType.kPosition); 
				break;
			case CLOSE:
				pivotController.setReference(Constants.Pivot.closePosition, CANSparkFlex.ControlType.kPosition); 
				break;
			default:
				break;
			}
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Pivot position", pivotEncoder.getPosition());


		if(position != ShooterPositions.NONE) {
			setFieldRelativePosition();
		} else {
			// autoAdjust(GlobalVariables.distanceToSpeaker);
		}
	}
}
