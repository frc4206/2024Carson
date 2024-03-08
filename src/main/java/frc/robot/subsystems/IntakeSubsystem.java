// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.robot.Constants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private CANSparkFlex intakeMotor = new CANSparkFlex(Constants.Intake.intakeDriveMotorID, MotorType.kBrushless);
	private CANSparkFlex intakeFollowerMotor = new CANSparkFlex(Constants.Intake.intakeFollowerMotorID, MotorType.kBrushless);

	public IntakeSubsystem() { 
		intakeFollowerMotor.follow(intakeMotor);
	}

	public void intakeGo(double setSpeed) {
		setMotorSpeed(intakeMotor, setSpeed);
	}

	@Override
	public void periodic() {}
}
