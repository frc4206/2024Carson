// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkConfiguration;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ConveyorSubsystem extends SubsystemBase implements SparkDefaultMethods {
  	private CANSparkFlex conveyorMotor = new CANSparkFlex(Constants.Conveyor.conveyorMotorID, MotorType.kBrushless);
	private RelativeEncoder conveyorEncoder = conveyorMotor.getEncoder();
	private SparkPIDController conveyorPIDController = conveyorMotor.getPIDController();
  	private DigitalInput conveyorBeamBreak = new DigitalInput(Constants.Conveyor.conveyerBeamBreakID);
	SparkConfiguration conveyorConfig;

  	public ConveyorSubsystem() {
		conveyorConfig = new SparkConfiguration(
			true,
			false,
			conveyorMotor,
			Constants.Conveyor.conveyorInverted,
			IdleMode.kBrake,
			40,
			conveyorEncoder,
			conveyorPIDController,
			Constants.Conveyor.conveyorkP,
			Constants.Conveyor.conveyorkI,
			Constants.Conveyor.conveyorkIzone,
			Constants.Conveyor.conveyorkD,
			0,
			Constants.Conveyor.conveyorMaxVelo,
			Constants.Conveyor.conveyorMaxAcc,
			Constants.Conveyor.conveyorMaxError
		);
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
			GlobalVariables.pieceReady = true;
		}
		return !conveyorBeamBreak.get();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("conveyorPosition", conveyorEncoder.getPosition());
	}
}
