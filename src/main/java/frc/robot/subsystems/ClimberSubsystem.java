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
  private CANSparkFlex climberLeaderMotor = new CANSparkFlex(Constants.Climber.climberLeaderMotorID, MotorType.kBrushless);
  //private CANSparkFlex climberFollowerMotor = new CANSparkFlex(Constants.Climber.climberFollowerID, MotorType.kBrushless);
  //private CANSparkFlex climberLeadMotor = new CANSparkFlex(Constants.Climber.climberLeaderMotorID, MotorType.kBrushless);
  private CANSparkFlex climberFollowerMotor = new CANSparkFlex(Constants.Climber.climberFollowerID, MotorType.kBrushless);

  private SparkPIDController climbLeadPid;
  public RelativeEncoder climbLeadEncoder; /* top encoder */
  //private RelativeEncoder climbBottomEncoder; /* bottom encoder */
  PWM servoRight = new PWM(Constants.Climber.servoRightID);
  PWM servoLeft = new PWM(Constants.Climber.servoLeftID);

  //private DigitalInput TopClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);
  //private DigitalInput BottomClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);

  /* Method Constructor */
  public ClimberSubsystem() {
    climberLeaderMotor.restoreFactoryDefaults();
    climberFollowerMotor.restoreFactoryDefaults();
    climberLeaderMotor.setInverted(false);
    climberFollowerMotor.setInverted(false);

    climberFollowerMotor.follow(climberLeaderMotor);

    climbLeadEncoder = climberLeaderMotor.getEncoder();
    climbLeadPid = climberLeaderMotor.getPIDController();
    
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
    climberLeaderMotor.set(0);
    //climberFollowerMotor.set(0);
  }

  public void climbUP() {
    climberLeaderMotor.set(0.3);
    //climberFollowerMotor.set(0.8);
  }

  public void climbDOWN() {
    climberLeaderMotor.set(-0.3);
    //climberFollowerMotor.set(-0.8);
  }

  public void GoToSetpoint(double setpoint) {
    climbLeadPid.setReference(setpoint, ControlType.kPosition, 0);
  }

  public void setPosition(double pos) {
    servoRight.setPosition(pos);
    servoLeft.setPosition(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(TopClimberLimitSwitch.get()) {
    //  climbLeadEncoder.setPosition(0);
    //}
    //if(BottomClimberLimitSwitch.get()) {
    //  climbLeadEncoder.setPosition(Constants.Climber.climberResetPosition);
    //} 
  }
}
