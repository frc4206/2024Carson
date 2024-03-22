// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.COMBOS;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.GlobalVariables;
import frc.robot.commands.Conveyor.ConveyorToDuty;
import frc.robot.commands.Intake.IntakeToDuty;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends ParallelCommandGroup {
  public Shoot(ConveyorSubsystem conveyor, IntakeSubsystem intake) {
    addCommands(
      new ConveyorToDuty(conveyor, 0.9),
      new IntakeToDuty(intake, GlobalVariables.Flywheel.toAmpVelo ? 9/50 : 1)
    );
  }
}
