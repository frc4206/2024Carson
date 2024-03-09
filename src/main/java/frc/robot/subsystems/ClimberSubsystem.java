// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkConfiguration;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.commands.Climber.RunServosCommand;
import frc.robot.commands.Climber.ClimberLeft.RunServoLeftCommand;
import frc.robot.commands.Climber.ClimberRight.RunServoRightCommand;

<<<<<<< HEAD
<<<<<<< HEAD
public class ClimberSubsystem extends SubsystemBase {
	public static CANSparkFlex climbLeadMotor = new CANSparkFlex(Constants.Climber.climberRightLeadID, MotorType.kBrushless);
	public static CANSparkFlex climbFollowMotor = new CANSparkFlex(Constants.Climber.climberLeftFollowID, MotorType.kBrushless);
	private SparkPIDController climbLeadPid;
	public RelativeEncoder climbLeadEncoder;
	public XboxController climberControl;
	private double motorRunSpeedLeft;
	private double motorRunSpeedRight;

	static PWM servoRight = new PWM(0);
	static PWM servoLeft = new PWM(1);

	public ClimberSubsystem(XboxController controller) {
		climbLeadMotor.restoreFactoryDefaults();
		climbFollowMotor.restoreFactoryDefaults();
		climbLeadMotor.setIdleMode(IdleMode.kBrake);
		climbLeadMotor.setInverted(false);
		//climbFollowMotor.follow(climbLeadMotor, true);

		climbLeadEncoder = climbLeadMotor.getEncoder();
		climbLeadPid = climbLeadMotor.getPIDController();
		
		climbLeadPid.setFeedbackDevice(climbLeadEncoder);
		climbLeadPid.setP(Constants.Climber.climberkP);
		climbLeadPid.setI(Constants.Climber.climberkI);
		climbLeadPid.setIZone(Constants.Climber.climberkIZone);
		climbLeadPid.setD(Constants.Climber.climberkD);
		climbLeadPid.setSmartMotionMaxVelocity(Constants.Climber.climberMaxVelo, 0);
		climbLeadPid.setSmartMotionMinOutputVelocity(-Constants.Climber.climberMaxVelo, 0);
		climbLeadPid.setSmartMotionMaxAccel(Constants.Climber.climberMaxAcc, 0);
		climbLeadPid.setSmartMotionAllowedClosedLoopError(Constants.Climber.climberAllowedError, 0);

		climbLeadMotor.burnFlash();
		climbFollowMotor.burnFlash();

		climberControl = controller;

		motorRunSpeedLeft = climberControl.getLeftTriggerAxis();
		motorRunSpeedRight = climberControl.getRightTriggerAxis();
  	}

	/* Stop climber motors voids */
	public void climbSTOP() {
		climbLeadMotor.set(0);
		climbFollowMotor.set(0);
		new RunServosCommand(this, Climber.servoPosLeftEngage, Climber.servoPosRightEngage);
	}

	public void climbSTOPLeft() {
		climbFollowMotor.set(0);
		new RunServoLeftCommand(this, Climber.servoPosLeftEngage);
	}

	public void climbSTOPRight() {
		climbLeadMotor.set(0);
		new RunServoRightCommand(this, Climber.servoPosRightEngage);
	}

	/* Run climber motors up voids */
	public void climberUP() {
		new RunServosCommand(this, Climber.servoPosLeftDisEngage, Climber.servoPosRightDisEngage);

		// Motor speed depends on which trigger is held down more.
		climbLeadMotor.set(climberControl.getLeftTriggerAxis());
		climbFollowMotor.set(-climberControl.getLeftTriggerAxis());
	}

	public void climbUPLeft() {
		new RunServoLeftCommand(this, Climber.servoPosLeftDisEngage);
		climbFollowMotor.set(climberControl.getLeftTriggerAxis());
	}

	public void climbUPRight() { 
		new RunServoRightCommand(this, Climber.servoPosRightDisEngage);
		climbLeadMotor.set(-climberControl.getRightTriggerAxis());
	}

	/* Run climber motors down voids */
	public void climbDOWN() {
		new RunServosCommand(this, Climber.servoPosLeftDisEngage, Climber.servoPosRightDisEngage);
		climbLeadMotor.set(-0.2);
		climbFollowMotor.set(0.2);
	}

	public void climbDOWNLeft() { 
		new RunServoLeftCommand(this, Climber.servoPosLeftDisEngage);
		climbFollowMotor.set(-0.2);
	}

	public void climbDOWNRight() {
		new RunServoRightCommand(this, Climber.servoPosRightDisEngage);
		climbLeadMotor.set(0.2);
	}

	/* PID setpoint set */
	public void GoToSetpoint(double setpoint) { 
		climbLeadPid.setReference(setpoint, ControlType.kPosition, 0); 
	}

	/* Servo set positions methods */
	public void setPositionBoth(double leftPos, double rightPos) {
		servoLeft.setPosition(leftPos);
		servoRight.setPosition(rightPos);
	}

	public void setPositionRight(double pos) {
		servoRight.setPosition(pos);
	}

	public void setPositionLeft(double pos) { 
		servoLeft.setPosition(pos); 
=======
public class ClimberSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private CANSparkFlex climberMotor;
	private RelativeEncoder climberEncoder;
	private SparkPIDController climberPIDController;
	private PWM servo;
	SparkConfiguration climberConfig;

	public ClimberSubsystem(int canID, boolean motorisInverted, int currentLimit, int servoID) {
        climberMotor = new CANSparkFlex(canID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        climberPIDController = climberMotor.getPIDController();
		servo = new PWM(servoID);

		climberConfig = new SparkConfiguration(
			true,
			false,
			climberMotor, 
			motorisInverted, 
			IdleMode.kBrake, 
			currentLimit, 
			climberEncoder, 
			climberPIDController, 
			Constants.Climber.climberkP, 
			Constants.Climber.climberkI, 
			Constants.Climber.climberkIZone, 
			Constants.Climber.climberkD, 
			0, 
			Constants.Climber.climberMaxVelo, 
			Constants.Climber.climberMaxAcc, 
			Constants.Climber.climberAllowedError
		);
  	}

	public void climbToPosition(double setpoint){
		motorGoToPosition(climberPIDController, setpoint);
=======
public class ClimberSubsystem extends SubsystemBase implements SparkDefaultMethods {
	private CANSparkFlex climberMotor;
	private RelativeEncoder climberEncoder;
	private SparkPIDController climberPIDController;
	private PWM servo;
	SparkConfiguration climberConfig;

	public ClimberSubsystem(int canID, boolean motorisInverted, int currentLimit, int servoID) {
        climberMotor = new CANSparkFlex(canID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        climberPIDController = climberMotor.getPIDController();
		servo = new PWM(servoID);

		climberConfig = new SparkConfiguration(
			true,
			false,
			climberMotor, 
			motorisInverted, 
			IdleMode.kBrake, 
			currentLimit, 
			climberEncoder, 
			climberPIDController, 
			Constants.Climber.climberkP, 
			Constants.Climber.climberkI, 
			Constants.Climber.climberkIZone, 
			Constants.Climber.climberkD, 
			0, 
			Constants.Climber.climberMaxVelo, 
			Constants.Climber.climberMaxAcc, 
			Constants.Climber.climberAllowedError
		);
  	}

	public void climbToPosition(double setpoint){
		motorGoToPosition(climberPIDController, setpoint);
	}

	public void climbToDuty(double setDuty){
		setMotorSpeed(climberMotor, setDuty);
	}

	public void setPosition(double pos){
		servo.setPosition(pos);
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
	}

	public void climbToDuty(double setDuty){
		setMotorSpeed(climberMotor, setDuty);
	}

	public void setPosition(double pos){
		servo.setPosition(pos);
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
	}

	// public void setPosition(double pos) {
	// 	servoLeft.setPosition(pos);
	// 	servoRight.setPosition(pos);
	// }

	@Override
<<<<<<< HEAD
<<<<<<< HEAD
	public void periodic() {
		// if(TopClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(0);
		// }
		// if(BottomClimberLimitSwitch.get()) {
		//  climbLeadEncoder.setPosition(Constants.Climber.climberResetPosition);
		// }

		// SmartDashboard.putNumber("Climber Right Position", climbRightLeadEncoder.getPosition());
		// SmartDashboard.putNumber("Climber Left Position", climbLeftFollowEncoder.getPosition());

		// SmartDashboard.putNumber("servo Right", servoRight.getPosition());
		// SmartDashboard.putNumber("servo Left", servoLeft.getPosition());
	}
=======
	public void periodic() {}
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
=======
	public void periodic() {}
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
}
