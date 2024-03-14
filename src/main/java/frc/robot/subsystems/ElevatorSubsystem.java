// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.SparkConfiguration;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase implements SparkDefaultMethods {
  private CANSparkFlex elevatorLeader = new CANSparkFlex(Constants.Elevator.elevatorLeaderID, MotorType.kBrushless);
  private RelativeEncoder elevatorLeaderEncoder = elevatorLeader.getEncoder();
  private SparkPIDController elevatorLeaderPIDController = elevatorLeader.getPIDController();
  private CANSparkFlex elevatorFollower = new CANSparkFlex(Constants.Elevator.elevatorFollowerID, MotorType.kBrushless);
  private RelativeEncoder elevatorFollowEncoder = elevatorFollower.getEncoder();
  private SparkPIDController elevatorFollowerPIDController = elevatorFollower.getPIDController();
  SparkConfiguration leaderConfig;
  SparkConfiguration followerConfig;

  public ElevatorSubsystem() {
    leaderConfig = new SparkConfiguration(
      true,
      false,
      elevatorLeader, 
      Constants.Elevator.elevatorLeaderisInverted, 
      IdleMode.kBrake, 
      40, 
      elevatorFollowEncoder, 
      elevatorFollowerPIDController, 
      Constants.Elevator.elevatorkP, 
      Constants.Elevator.elevatorkI, 
      Constants.Elevator.elevatorkIZone, 
      Constants.Elevator.elevatorkD, 
      0, 
      Constants.Elevator.elevatorMaxVelo, 
      Constants.Elevator.elevatorMaxAcc, 
      Constants.Elevator.elevatorMaxError
    );
    followerConfig = new SparkConfiguration(
      true,
      false,
      elevatorFollower, 
      Constants.Elevator.elevatorFollowisInverted, 
      IdleMode.kCoast, 
      40, 
      elevatorFollowEncoder, 
      elevatorFollowerPIDController, 
      Constants.Elevator.elevatorkP, 
      Constants.Elevator.elevatorkI, 
      Constants.Elevator.elevatorkIZone, 
      Constants.Elevator.elevatorkD, 
      0, 
      Constants.Elevator.elevatorMaxVelo, 
      Constants.Elevator.elevatorMaxAcc, 
      Constants.Elevator.elevatorMaxError
    );    
  }

	public void resetElevator(){
    resetMotor(elevatorLeaderEncoder);
    resetMotor(elevatorFollowEncoder);
  }

	public void elevatorToPercent(double percent){
    setMotorSpeed(elevatorLeader, percent);
    setMotorSpeed(elevatorFollower, percent);
  }

	public void GoToSetpoint(double setpoint) {
		motorGoToPosition(elevatorLeaderPIDController, setpoint);
		motorGoToPosition(elevatorFollowerPIDController, setpoint);
	}


  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevPosition", elevatorLeaderEncoder.getPosition());
  }
}
