// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.SetupNote;
import frc.robot.commands.Swerve.PID_to_game_Piece;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeCommand extends ParallelCommandGroup {
  /** Creates a new AutoIntakeCommand. */
  public AutoIntakeCommand(ConveyorSubsystem conveyor, IntakeSubsystem intake, SwerveSubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetupNote(conveyor, intake), new PID_to_game_Piece(swerve, false, true, false, 10));
  }
}
