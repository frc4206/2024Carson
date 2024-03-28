// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private CANSparkFlex intakeMotor = new CANSparkFlex(Constants.Intake.intakeDriveMotorID, MotorType.kBrushless);
	SparkConfig intakeConfig;

	public IntakeSubsystem() {
		intakeConfig = Constants.Intake.intakeConfig;
		intakeConfig.configureController(intakeMotor);
		intakeConfig.applyMotorConfigurations();
	}

	public void intakeToDuty(double setSpeed) {
		motorToDuty(intakeMotor, setSpeed);
	}

	@Override
	public void periodic() {
		GlobalVariables.Intake.intakeDuty = intakeMotor.getAppliedOutput();
	}
}
