// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		setUseTiming(false);
		Logger.start(); 
		m_robotContainer = new RobotContainer();
		Mod0.angleOffset = m_robotContainer.m_swerveSubsystem.mSwerveMods[0].getCanCoder().getDegrees();
		Mod1.angleOffset = m_robotContainer.m_swerveSubsystem.mSwerveMods[1].getCanCoder().getDegrees();
		Mod2.angleOffset = m_robotContainer.m_swerveSubsystem.mSwerveMods[2].getCanCoder().getDegrees();
		Mod3.angleOffset = m_robotContainer.m_swerveSubsystem.mSwerveMods[3].getCanCoder().getDegrees();
		m_robotContainer.m_Limelight.ChangePipelines(2);
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		GlobalVariables.isEnabled = false;
		for (SwerveModule mod : m_robotContainer.m_swerveSubsystem.mSwerveMods){
			mod.ctreConfigs.swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			mod.mDriveMotor.getConfigurator().apply(mod.ctreConfigs.swerveDriveFXConfig);
		}
	}

	@Override
	public void disabledPeriodic() {
		GlobalVariables.alliance = DriverStation.getAlliance().get();
	}

	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		GlobalVariables.isEnabled = true;
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
		m_autonomousCommand.cancel();
		}
		for (SwerveModule mod : m_robotContainer.m_swerveSubsystem.mSwerveMods){
			mod.ctreConfigs.swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			mod.mDriveMotor.getConfigurator().apply(mod.ctreConfigs.swerveDriveFXConfig);
		}
		GlobalVariables.isEnabled = true;
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {}
}