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

public class VortexElevatorSubsystem extends SubsystemBase {

  /* Variables */
  private CANSparkFlex elevatorLeader = new CANSparkFlex(Constants.Elevator.ElevatorLeaderID, MotorType.kBrushless);
  private CANSparkFlex elevatorFollower = new CANSparkFlex(Constants.Elevator.ElevatorFollowerID, MotorType.kBrushless);

  private DigitalInput elevatorTopLimitSwitch = new DigitalInput(Constants.Elevator.ElevatorTopLimitSwitch);
  private DigitalInput elevatorBottomLimitSwitch = new DigitalInput(Constants.Elevator.ElevatorBottomLimitSwitch);
  
  private SparkPIDController elevatorLeadPid;
  private RelativeEncoder elevatorLeadEncoder;

  /* Method Constructor */
  public VortexElevatorSubsystem() {
    elevatorLeader.restoreFactoryDefaults();
    elevatorLeader.setInverted(false);
    elevatorFollower.setInverted(false);
    

    elevatorLeader.restoreFactoryDefaults();
    elevatorFollower.restoreFactoryDefaults();
    elevatorLeader.setInverted(false);
    elevatorFollower.setInverted(false);

    elevatorLeadEncoder = elevatorLeader.getEncoder();
    elevatorLeadPid = elevatorLeader.getPIDController();
    
    elevatorLeadPid.setFeedbackDevice(elevatorLeadEncoder);
    elevatorLeadPid.setP(Constants.Elevator.elevKP);
    elevatorLeadPid.setI(Constants.Elevator.elevKI);
    elevatorLeadPid.setD(Constants.Elevator.elevKD);

    elevatorFollower.follow(elevatorLeader);
  }

  public void elevatorSTOP() {
    elevatorFollower.follow(elevatorLeader);
    elevatorLeader.set(Constants.Elevator.elevStopSpeed);
  }

  public void elevatorUP() {
    elevatorFollower.follow(elevatorLeader);
    elevatorLeader.set(Constants.Elevator.elevUpSpeed);
  }

  public void elevatorDOWN() {
    elevatorFollower.follow(elevatorLeader);
    elevatorLeader.set(Constants.Elevator.elevDownSpeed);
  }

  public void GoToSetpoint(double setpoint) {
    elevatorLeadPid.setReference(setpoint, ControlType.kPosition, 0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(elevatorBottomLimitSwitch.get()) {
      elevatorLeadEncoder.setPosition(0);
    }
    /* Get value from testing!! */
    if(elevatorTopLimitSwitch.get()) {
      elevatorLeadEncoder.setPosition(7.5);
    }
  }
}
