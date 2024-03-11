// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Pivot.PivotToSpeaker;
import frc.robot.commands.Shooter.ShooterToSpeaker;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CarriageToSpeaker extends ParallelCommandGroup {
  public CarriageToSpeaker(FlywheelSubsystem flywheel, PivotSubsystem pivot) {
    addCommands(
      new PivotToSpeaker(pivot),
      new ShooterToSpeaker(flywheel)
    );
  }
}
