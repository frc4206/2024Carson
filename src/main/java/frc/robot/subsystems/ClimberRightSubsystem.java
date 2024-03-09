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

@Deprecated
public class ClimberRightSubsystem extends SubsystemBase {

  /* Variables */
  public static CANSparkFlex climberRightLead = ClimberSubsystem.climbLeadMotor; // new CANSparkFlex(Constants.Climber.climberRightLeadID, MotorType.kBrushless);
//X  public static CANSparkFlex climberLeftFollow = new CANSparkFlex(Constants.Climber.climberLeftFollowID, MotorType.kBrushless);

  private SparkPIDController climbRightPid;
  public RelativeEncoder climbRightEncoder; /* top encoder */
  //private RelativeEncoder climbBottomEncoder; /* bottom encoder */

  public XboxController joystick; 

  PWM servoRight = ClimberSubsystem.servoRight;
//X  PWM servoLeft = new PWM(1);

  //private DigitalInput TopClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);
  //private DigitalInput BottomClimberLimitSwitch = new DigitalInput(Constants.Climber.climberLimitSwitch);

  public ClimberRightSubsystem(XboxController joy) {
//X    climberLeftFollow.restoreFactoryDefaults();
    climberRightLead.restoreFactoryDefaults();
 //X   climberLeftFollow.setInverted(true);
    climberRightLead.setInverted(false);
    climberRightLead.setIdleMode(IdleMode.kBrake);
    joystick = joy;
    //climberRightLead.follow(climberLeftFollow);

    climbRightEncoder = climberRightLead.getEncoder();
    climbRightEncoder.setPosition(0);
    climbRightPid = climberRightLead.getPIDController();
    
    climbRightPid.setFeedbackDevice(climbRightEncoder);
    climbRightPid.setP(0.02);
    climbRightPid.setI(9e-8);
    climbRightPid.setD(0.0);
    climbRightPid.setFF(0.0);
    climbRightPid.setSmartMotionMaxVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecondfast, 0);
    climbRightPid.setSmartMotionMinOutputVelocity(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 0);
    climbRightPid.setSmartMotionMaxAccel(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 0);
    climbRightPid.setSmartMotionAllowedClosedLoopError(5, 0);

    servoRight.setPosition(0);
  }

  public void climbSTOP() {
//X    climberLeftFollow.set(0);
    climberRightLead.set(0);
  }

  public void climbUPRight() {
    climberRightLead.set(0.2);
  }

  public void climbDOWNRight() {
    climberRightLead.set(-0.2);
  }

//X  public void climbUPLeft() {
//X    climberLeftFollow.set(0.2);
//X  }

//X  public void climbDOWNLeft() {
//X    climberLeftFollow.set(-0.2);
//X  }

  public void GoToSetpoint(double setpoint) {
    climbRightPid.setReference(setpoint, ControlType.kPosition, 0);

    /*// Raise climber upwards
    if(climbRightEncoder.getPosition() < setpoint) {
      climberRightLead.set(speed);
    } else {
      climberRightLead.set(0);
    }*/
  }

  public void setPositionRight(double pos) {
    servoRight.setPosition(pos);
  }

//X  public void setPositionLeft(double pos) {
//X    servoLeft.setPosition(pos);
//X  }

//X  public void runServoLeft(double speed) {
//X    servoLeft.setSpeed(speed);
//X  }
  public void RunBothMotorsUp() {
    climberRightLead.set(joystick.getLeftTriggerAxis()); 
//X    climberLeftFollow.set(joystick.getLeftTriggerAxis());
  }

  public void RunBothMotorsDown() {
    //climberLeftFollow.setInverted(false);
    //climberRightLead.setInverted(true);`
    climberRightLead.set(-joystick.getRightTriggerAxis()); 
//X    climberLeftFollow.set(-joystick.getRightTriggerAxis()); 

  }

  public void RunRight() {
    climberRightLead.set(joystick.getRawAxis(5)); 
  }

 //x public void RunLeft() {
//X    climberLeftFollow.set(joystick.getRawAxis(1));
 //x }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(TopClimberLimitSwitch.get()) {
    //  climbRightEncoder.setPosition(0);
    //}
    //if(BottomClimberLimitSwitch.get()) {
    //  climbRightEncoder.setPosition(Constants.Climber.climberResetPosition);
    //}
    SmartDashboard.putNumber("Climber Right Position", climbRightEncoder.getPosition());
    SmartDashboard.putNumber("Rightpower", climberRightLead.getAppliedOutput());
//X    SmartDashboard.putNumber("Climber Left Position", climberLeftFollow.getEncoder().getPosition());
    SmartDashboard.putNumber("Servo Right Position", servoRight.getPosition());
//X    SmartDashboard.putNumber("Servo Left Position", servoLeft.getPosition());
  }
}
