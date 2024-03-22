// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ConveyorSubsystem extends SubsystemBase implements SparkDefaultMethods {
  	private CANSparkFlex conveyorMotor = new CANSparkFlex(Constants.Conveyor.conveyorMotorID, MotorType.kBrushless);
	private RelativeEncoder conveyorEncoder = conveyorMotor.getEncoder();
	private SparkPIDController conveyorPIDController = conveyorMotor.getPIDController();
	SparkConfig conveyorConfig;

	private DigitalInput conveyorBeamBreak = new DigitalInput(Constants.Conveyor.conveyerBeamBreakID);

  	public ConveyorSubsystem() {
		conveyorConfig = Constants.Conveyor.conveyorConfig;
		conveyorConfig.configureController(conveyorMotor, conveyorEncoder, conveyorPIDController);
		conveyorConfig.applyAllConfigurations();
	}

	public void conveyorToPosition(double desiredPosition){
		motorToPosition(conveyorPIDController, desiredPosition);
	}

	public void conveyorToDuty(double desiredDuty){
		motorToDuty(conveyorMotor, desiredDuty);
	}

	public void resetConveyorEncoder(){
		resetEncoder(conveyorEncoder);
	}

	@Override
	public void periodic() {
		GlobalVariables.Conveyor.beamBroken = !conveyorBeamBreak.get();
		GlobalVariables.Conveyor.conveyorPosition = conveyorEncoder.getPosition();
		SmartDashboard.putBoolean("PIECEREADY", GlobalVariables.Conveyor.beamBroken);
		SmartDashboard.putNumber("conveyorPosition", GlobalVariables.Conveyor.conveyorPosition);
	}	
}
