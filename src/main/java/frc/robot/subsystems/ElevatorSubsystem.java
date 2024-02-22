// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkFlex elevatorLeader = new CANSparkFlex(Constants.Elevator.elevatorLeaderID, MotorType.kBrushless);
  private RelativeEncoder elevatorLeaderEncoder = elevatorLeader.getEncoder();
  private SparkPIDController elevatorLeaderPIDController = elevatorLeader.getPIDController();
  private CANSparkFlex elevatorFollower = new CANSparkFlex(Constants.Elevator.elevatorFollowerID, MotorType.kBrushless);

  private DigitalInput elevatorTopLimitSwitch = new DigitalInput(Constants.Elevator.elevatorTopLimitSwitch);
  private DigitalInput elevatorBottomLimitSwitch = new DigitalInput(Constants.Elevator.elevatorBottomLimitSwitch);

  public ElevatorSubsystem() {
    elevatorLeader.restoreFactoryDefaults();
    elevatorFollower.restoreFactoryDefaults();
    elevatorLeader.setInverted(Constants.Elevator.elevatorLeaderisInverted);
    elevatorFollower.setInverted(Constants.Elevator.elevatorFollowisInverted);
    
    elevatorLeaderPIDController.setFeedbackDevice(elevatorLeaderEncoder);
    elevatorLeaderPIDController.setP(Constants.Elevator.elevKP);
    elevatorLeaderPIDController.setI(Constants.Elevator.elevKI);
    elevatorLeaderPIDController.setD(Constants.Elevator.elevKD);

    elevatorLeader.burnFlash();
    elevatorFollower.burnFlash();
  }

  public void elevatorSTOP() {
    elevatorLeader.set(Constants.Elevator.elevStopSpeed);
    elevatorFollower.set(Constants.Elevator.elevStopSpeed);
  }

  public void elevatorUP() {
    elevatorLeader.set(Constants.Elevator.elevUpSpeed);
    elevatorFollower.set(Constants.Elevator.elevUpSpeed);
  }

  public void elevatorDOWN() {
    elevatorLeader.set(Constants.Elevator.elevDownSpeed);
    elevatorFollower.set(Constants.Elevator.elevDownSpeed);
  }

  public void GoToSetpoint(double setpoint) {
    elevatorLeaderPIDController.setReference(setpoint, ControlType.kPosition, 0);
  }


  @Override
  public void periodic() {
    if(!elevatorBottomLimitSwitch.get()) {
      elevatorLeaderEncoder.setPosition(0);
    }
    if(!elevatorTopLimitSwitch.get()) {
      elevatorLeaderEncoder.setPosition(Constants.Elevator.elevResetPosition);
    }
  }
}
