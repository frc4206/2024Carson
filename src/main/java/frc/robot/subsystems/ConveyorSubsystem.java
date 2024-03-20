// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

public class ConveyorSubsystem extends SubsystemBase implements SparkDefaultMethods {
  	private CANSparkFlex conveyorMotor;
	private RelativeEncoder conveyorEncoder;
	private SparkPIDController conveyorPIDController;
	SparkConfig conveyorConfig;
	ControllerConfig controllerConfig;

	private DigitalInput conveyorBeamBreak = new DigitalInput(Constants.Conveyor.conveyerBeamBreakID);

  	public ConveyorSubsystem() {
		controllerConfig = new ControllerConfig(Constants.Conveyor.conveyorMotorID, conveyorMotor, conveyorEncoder, conveyorPIDController);
		conveyorMotor = controllerConfig.getMotor();
		conveyorEncoder = controllerConfig.getEncoder();
		conveyorPIDController = controllerConfig.getPIDController();

		conveyorConfig = new SparkConfig(
			new FeedbackConfig(-1, 1, Constants.Conveyor.conveyorMaxVelo, Constants.Conveyor.conveyorMaxAcc, Constants.Conveyor.conveyorMaxError),
			new MotorConfig(Constants.Conveyor.conveyorMotorID, Constants.Conveyor.conveyorInverted, IdleMode.kBrake, 40),
			new PIDConfig(Constants.Conveyor.conveyorkP, Constants.Conveyor.conveyorkI, Constants.Conveyor.conveyorkIzone, Constants.Conveyor.conveyorkD, 0),
			conveyorMotor,
			conveyorEncoder,
			conveyorPIDController,
			false,
			false
		);
		conveyorConfig.applyConfig();
	}

	public void conveyorGoToPosition(double desiredPosition){
		motorGoToPosition(conveyorPIDController, desiredPosition);
	}

	public void conveyorToDuty(double desiredDuty){
		setMotorSpeed(conveyorMotor, desiredDuty);
	}

	public void resetConveyor(){
		resetMotor(conveyorEncoder);
	}

	public boolean hasNote() {
		if (!conveyorBeamBreak.get()) {
			SmartDashboard.putBoolean("PIECEREADY", !conveyorBeamBreak.get());
		}
		return !conveyorBeamBreak.get();
	}

	@Override
	public void periodic() {
		GlobalVariables.Conveyor.beamBroken = conveyorBeamBreak.get();

		SmartDashboard.putNumber("conveyorPosition", conveyorEncoder.getPosition());
		SmartDashboard.putBoolean("beam broken", GlobalVariables.Conveyor.beamBroken);
	}	
}
