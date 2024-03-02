// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbRightSubystem extends SubsystemBase {
	private CANSparkFlex climberRightMotor = new CANSparkFlex(Constants.Climber.climberRightLeadID, MotorType.kBrushless);
  private RelativeEncoder climberRightEncoder = climberRightMotor.getEncoder();
  private SparkPIDController climberRightPIDController = climberRightMotor.getPIDController();

  private PWM servoRight = new PWM(Constants.Climber.servoRightID);

  public ClimbRightSubystem() {
		climberRightMotor.restoreFactoryDefaults();
		climberRightMotor.setIdleMode(IdleMode.kBrake);
		climberRightMotor.setInverted(false);
		climberRightMotor.setSmartCurrentLimit(40);
		climberRightMotor.burnFlash();
		
		climberRightEncoder.setPosition(0);
		climberRightPIDController.setFeedbackDevice(climberRightEncoder);

		climberRightPIDController.setP(Constants.Climber.climberkP);
		climberRightPIDController.setI(Constants.Climber.climberkI);
		climberRightPIDController.setIZone(Constants.Climber.climberkIZone);
		climberRightPIDController.setD(Constants.Climber.climberkD);
		climberRightPIDController.setOutputRange(-1, 1, 0);
		climberRightPIDController.setSmartMotionMaxVelocity(Constants.Climber.climberMaxVelo, 0);
		climberRightPIDController.setSmartMotionMaxAccel(Constants.Climber.climberMaxAcc, 0);
		climberRightPIDController.setSmartMotionAllowedClosedLoopError(Constants.Climber.climberAllowedError, 0);
  }

  public void climbToPosition(double setpoint){
    climberRightPIDController.setReference(setpoint, ControlType.kPosition);
  }

  public void climbSTOP(){
    climberRightMotor.set(0);
  }

  public void climberUP(){
    climberRightMotor.set(0.2);
  }

  public void climbDOWN(){
    climberRightMotor.set(-0.2);
  }

  public void setPosition(double pos){
    servoRight.setPosition(pos);
  }

  @Override
  public void periodic() {}
}
