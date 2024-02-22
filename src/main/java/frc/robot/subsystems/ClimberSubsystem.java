// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  /* Variables */
  private CANSparkFlex climberLeftLead = new CANSparkFlex(Constants.Climber.climberLeftLeadID, MotorType.kBrushless);
  private CANSparkFlex climberRightFollow = new CANSparkFlex(Constants.Climber.climberRightFollowID, MotorType.kBrushless);

  private SparkPIDController climbLeadPid;
  private RelativeEncoder climbLeadEncoder; /* top encoder */
  //private RelativeEncoder climbBottomEncoder; /* bottom encoder */
  PWM servoRight = new PWM(0);
  PWM servoLeft = new PWM(1);

  private DigitalInput climberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);

  /* Method Constructor */
  public ClimberSubsystem() {
    climberLeftLead.restoreFactoryDefaults();
    climberRightFollow.restoreFactoryDefaults();
    climberLeftLead.setInverted(false);
    climberRightFollow.setInverted(false);

    //climberRightFollow.follow(climberLeftLead);

    climbLeadEncoder = climberLeftLead.getEncoder();
    climbLeadPid = climberLeftLead.getPIDController();
    
    climbLeadPid.setFeedbackDevice(climbLeadEncoder);
    climbLeadPid.setP(0.02);
    climbLeadPid.setI(9e-8);
    climbLeadPid.setD(0.0);
    //climbLeadPid.setFF(0.0);
    //climbLeadPid.setSmartMotionMaxVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecondfast, 0);
    //climbLeadPid.setSmartMotionMinOutputVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 0);
    //climbLeadPid.setSmartMotionMaxAccel(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 0);
    //climbLeadPid.setSmartMotionAllowedClosedLoopError(5, 0);

    
  }

  public void climbSTOP() {
    climberLeftLead.set(0);
    climberRightFollow.set(0);
  }

  public void climbUPRight() {
    climberRightFollow.set(0.2);
  }

  public void climbDOWNRight() {
    climberRightFollow.set(-0.2);
  }

  public void climbUPLeft() {
    climberLeftLead.set(0.2);
  }

  public void climbDOWNLeft() {
    climberLeftLead.set(-0.2);
  }

  public void GoToSetpoint(double setpoint) {
    climbLeadPid.setReference(setpoint, ControlType.kPosition, 0);
  }

  public void setPositionRight(double pos) {
    servoRight.setPosition(pos);
  }

  public void setPositionLeft(double pos) {
    servoLeft.setPosition(pos);
  }

  public void runServoLeft(double speed) {
    servoLeft.setSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(climberLimitSwitch.get()) {
      climbLeadEncoder.setPosition(0);
    }
    /*if(climberBottomSwitch.get()) {
      climbLeadEncoder.setPosition(7.5);
    } */
  }
}
