// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVariables;
import frc.robot.commands.Conveyor.ConveyorToDuty;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetupNote extends SequentialCommandGroup {
  public SetupNote(ConveyorSubsystem conveyor, IntakeSubsystem intake) {
    addCommands(
      new GoUntilNote(conveyor, intake).until(() -> GlobalVariables.Conveyor.beamBroken),
      // new ConveyerToPosition(conveyor, 18).withTimeout(2),
      // new ConveyerToPosition(conveyor, 17.5).withTimeout(2)
      new ParallelCommandGroup(
        new ConveyorToDuty(conveyor, 0.2),
        new IntakeToDuty(intake, 0.1)
      ).withTimeout(.85)
    );
  }
}
