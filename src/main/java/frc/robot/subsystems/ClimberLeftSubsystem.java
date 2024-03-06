// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberLeftSubsystem extends SubsystemBase {

  /* Variables */
//X  public static CANSparkFlex climberRightLead = new CANSparkFlex(Constants.Climber.climberRightLeadID, MotorType.kBrushless);
  public static CANSparkFlex climberLeftFollow = new CANSparkFlex(Constants.Climber.climberLeftFollowID, MotorType.kBrushless);

  private SparkPIDController climbLeftPid;
  public RelativeEncoder climbLeftEncoder; /* top encoder */
  //private RelativeEncoder climbBottomEncoder; /* bottom encoder */

  public XboxController joystick; 

//X  PWM servoRight = new PWM(0);
  PWM servoLeft = new PWM(1);

  //private DigitalInput TopClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);
  //private DigitalInput BottomClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);

  public ClimberLeftSubsystem(XboxController joy) {
    climberLeftFollow.restoreFactoryDefaults();
  //x  climberRightLead.restoreFactoryDefaults();
    climberLeftFollow.setInverted(true);
  //x  climberRightLead.setInverted(false);
    joystick = joy;
    //climberRightLead.follow(climberLeftFollow);
    climberLeftFollow.follow(ClimberRightSubsystem.climberRightLead, true);
    climberLeftFollow.setIdleMode(IdleMode.kBrake);

    climbLeftEncoder = climberLeftFollow.getEncoder();
    climbLeftEncoder.setPosition(0);
    climbLeftPid = climberLeftFollow.getPIDController();
    
    climbLeftPid.setFeedbackDevice(climbLeftEncoder);
    climbLeftPid.setP(0.02);
    climbLeftPid.setI(9e-8);
    climbLeftPid.setD(0.0);
    climbLeftPid.setFF(0.0);
    climbLeftPid.setSmartMotionMaxVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecondfast, 0);
    climbLeftPid.setSmartMotionMinOutputVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 0);
    climbLeftPid.setSmartMotionMaxAccel(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 0);
    climbLeftPid.setSmartMotionAllowedClosedLoopError(5, 0);
  }

  public void climbSTOP() {
    climberLeftFollow.set(0);
   //x climberRightLead.set(0);
  }

  //xpublic void climbUPRight() {
  //x  climberRightLead.set(0.2);
  //x}

  //xpublic void climbDOWNRight() {
 //x   climberRightLead.set(-0.2);
  //x}

  public void climbUPLeft() {
    climberLeftFollow.set(0.2);
  }

  public void climbDOWNLeft() {
    climberLeftFollow.set(-0.2);
  }

  public void GoToSetpoint(double setpoint) {
    climbLeftPid.setReference(setpoint, ControlType.kPosition, 0);
  }

//x  public void setPositionRight(double pos) {
    //xservoRight.setPosition(pos);
  //x}

  public void setPositionLeft(double pos) {
    servoLeft.setPosition(pos);
  }

  public void runServoLeft(double speed) {
    servoLeft.setSpeed(speed);
  }
  public void RunBothMotorsUp() {
    //xclimberRightLead.set(joystick.getLeftTriggerAxis(8)); 
    climberLeftFollow.set(joystick.getLeftTriggerAxis());
  }

  public void RunBothMotorsDown() {
    //climberLeftFollow.setInverted(false);
    //climberRightLead.setInverted(true);
    //climberRightLead.set(-joystick.getRightTriggerAxis()); 
    climberLeftFollow.set(-joystick.getRightTriggerAxis()); 

  }

  //xpublic void RunRight() {
    //xclimberRightLead.set(joystick.getRawAxis(5)); 
  //x}

  public void RunLeft() {
    climberLeftFollow.set(joystick.getRawAxis(1));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(TopClimberLimitSwitch.get()) {
    //  climbLeftEncoder.setPosition(0);
    //}
    //if(BottomClimberLimitSwitch.get()) {
    //  climbLeftEncoder.setPosition(Constants.Climber.climberResetPosition);
    //}
    //SmartDashboard.putNumber("Climber Right Position", climbLeftEncoder.getPosition());
    SmartDashboard.putNumber("Climber Left Position", climberLeftFollow.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Power", climberLeftFollow.getAppliedOutput());
//    SmartDashboard.putNumber("Servo Right Position", servoRight.getPosition());
    SmartDashboard.putNumber("Servo Left Position", servoLeft.getPosition());
  }
}
