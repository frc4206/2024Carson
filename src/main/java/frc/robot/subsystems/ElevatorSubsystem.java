// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.SparkConfiguration;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

/** Elevator is now no longer being used for 2024Carson. */
@Deprecated
public class ElevatorSubsystem extends SubsystemBase implements SparkDefaultMethods {
	// private CANSparkFlex elevatorLeader = new CANSparkFlex(Constants.Elevator.elevatorLeaderID, MotorType.kBrushless);
	// private RelativeEncoder elevatorLeaderEncoder = elevatorLeader.getEncoder();
	// private SparkPIDController elevatorLeaderPIDController = elevatorLeader.getPIDController();

	// private CANSparkFlex elevatorFollower = new CANSparkFlex(Constants.Elevator.elevatorFollowerID, MotorType.kBrushless);
	// private RelativeEncoder elevatorFollowEncoder = elevatorFollower.getEncoder();
	// private SparkPIDController elevatorFollowerPIDController = elevatorFollower.getPIDController();

	// SparkConfiguration leaderConfig;
	// SparkConfiguration followerConfig;

	private CANSparkFlex elevatorMotor;
	private RelativeEncoder elevatorEncoder;
	private SparkPIDController elevatorPIDController;
	SparkConfiguration elevConfig;

	public ElevatorSubsystem() {}

	public ElevatorSubsystem(int motorCanId, boolean invertMotor, IdleMode idleMode) {
		elevatorMotor = new CANSparkFlex(motorCanId, MotorType.kBrushless);
		elevatorMotor.setIdleMode(idleMode);
		elevatorMotor.setInverted(invertMotor);

		elevatorEncoder = elevatorMotor.getEncoder();
		elevatorPIDController = elevatorMotor.getPIDController();

		elevatorPIDController.setP(Constants.Elevator.elevatorkP);
		elevatorPIDController.setI(Constants.Elevator.elevatorkI);
		elevatorPIDController.setIZone(Constants.Elevator.elevatorkIZone);
		elevatorPIDController.setD(Constants.Elevator.elevatorkD);
		elevatorPIDController.setFF(0);
		elevatorPIDController.setSmartMotionMaxVelocity(Constants.Elevator.elevatorMaxVelo, motorCanId);
		elevatorPIDController.setSmartMotionMaxAccel(Constants.Elevator.elevatorMaxAcc, motorCanId);
		elevatorPIDController.setSmartMotionAllowedClosedLoopError(Constants.Elevator.elevatorMaxError, motorCanId);

		elevConfig = new SparkConfiguration(
			true,
			false,
			elevatorMotor,
			invertMotor,
			idleMode,
			40,
			elevatorEncoder,
			elevatorPIDController,
			Elevator.elevatorkP,
			Elevator.elevatorkI,
			Elevator.elevatorkIZone,
			Elevator.elevatorkD,
			0,
			Elevator.elevatorMaxVelo,
			Elevator.elevatorMaxAcc,
			Elevator.elevatorMaxError);

		//leaderConfig = new SparkConfiguration(
		// true,
		// false,
		// elevatorLeader, 
		// Constants.Elevator.elevatorLeaderisInverted, 
		// IdleMode.kBrake, 
		// 40, 
		// elevatorFollowEncoder, 
		// elevatorFollowerPIDController, 
		// Constants.Elevator.elevatorkP, 
		// Constants.Elevator.elevatorkI, 
		// Constants.Elevator.elevatorkIZone, 
		// Constants.Elevator.elevatorkD, 
		// 0, 
		// Constants.Elevator.elevatorMaxVelo, 
		// Constants.Elevator.elevatorMaxAcc, 
		// Constants.Elevator.elevatorMaxError
		// );
		// followerConfig = new SparkConfiguration(
		// true,
		// false,
		// elevatorFollower, 
		// Constants.Elevator.elevatorFollowisInverted, 
		// IdleMode.kCoast, 
		// 40, 
		// elevatorFollowEncoder, 
		// elevatorFollowerPIDController, 
		// Constants.Elevator.elevatorkP, 
		// Constants.Elevator.elevatorkI, 
		// Constants.Elevator.elevatorkIZone, 
		// Constants.Elevator.elevatorkD, 
		// 0, 
		// Constants.Elevator.elevatorMaxVelo, 
		// Constants.Elevator.elevatorMaxAcc, 
		// Constants.Elevator.elevatorMaxError
		// );
	}

	public void resetElevator() {
		resetMotor(elevatorEncoder);
	}

	public void elevatorToPercent(double percent) {
		setMotorSpeed(elevatorMotor, percent);
	}

	public void goToSetpoint(double setpoint) {
		motorGoToPosition(elevatorPIDController, setpoint);
	}


	@Override
	public void periodic() {
		SmartDashboard.putNumber("elevPosition", elevatorEncoder.getPosition());
	}
}
