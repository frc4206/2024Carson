// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Autos;
import frc.robot.commands.ChangePipelineCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Swerve.PID_DistanceOdometry;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.ZeroGyroCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Joystick driver = new Joystick(0);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final Limelight m_Limelight = new Limelight();
  private final SwerveSubsystem swerve = new SwerveSubsystem(m_Limelight);

  public final static edu.wpi.first.wpilibj.XboxController.Axis tAxis = XboxController.Axis.kLeftY;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  Joystick operator = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    boolean fieldRelative = true;
    boolean openLoop = true;
    swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    configureBindings();
  }

  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    new JoystickButton(driver, 3).onTrue(new ZeroGyroCommand(swerve));
    new JoystickButton(driver, 1).onTrue(new ChangePipelineCommand(m_Limelight, 0));
    new JoystickButton(driver, 2).onTrue(new ChangePipelineCommand(m_Limelight, 1));
    new JoystickButton(driver, 4).onTrue(new ChangePipelineCommand(m_Limelight, 2));
    new JoystickButton(driver, 5).whileTrue(new InstantCommand(() -> swerve.justTurn()));
    new JoystickButton(driver, 6).whileTrue(new PID_DistanceOdometry(m_Limelight, swerve, translationAxis, strafeAxis, rotationAxis, false, false, 1, 1, 0, 10));

    new JoystickButton(operator, 1).onTrue(new ChangePipelineCommand(m_Limelight, 0));
    new JoystickButton(operator, 2).onTrue(new ChangePipelineCommand(m_Limelight, 1));
    new JoystickButton(operator, 3).onTrue(new ChangePipelineCommand(m_Limelight, 2));
    // Schedule `exampleMehodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
