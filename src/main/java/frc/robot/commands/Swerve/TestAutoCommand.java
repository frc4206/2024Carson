// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChangePipelineCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoCommand extends SequentialCommandGroup {

  /** Creates a new TestAutoCommand. */
  public TestAutoCommand(SwerveSubsystem s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     new PID_DistanceOdometry2(s_Swerve, true, false, -6.6, -1, 0, 5),
     new PID_DistanceOdometry2(s_Swerve, true, false, -5, -5, 0, 10),
     new ChangePipelineCommand(s_Swerve.limelight, 1),
     new PID_to_game_Peice(s_Swerve, false, openLoop, false,5)
     );
  }
}
