// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ConveyorSubsystem extends SubsystemBase {
  	private CANSparkFlex conveyorMotor = new CANSparkFlex(Constants.Conveyor.conveyorMotorID, MotorType.kBrushless);
  	private DigitalInput conveyorBeamBreak = new DigitalInput(Constants.Conveyor.conveyerBeamBreakID);

  	public ConveyorSubsystem() {
    	conveyorMotor.setInverted(Constants.Conveyor.conveyorInverted);
  	}

  	public void conveyorTurn(double conveyorSpeed) {
    	conveyorMotor.set(conveyorSpeed);
  	}

	public boolean hasNote() {
		if (!conveyorBeamBreak.get()){
			GlobalVariables.pieceReady = true;
		}
		return !conveyorBeamBreak.get();
	}

	@Override
	public void periodic() {}
}
