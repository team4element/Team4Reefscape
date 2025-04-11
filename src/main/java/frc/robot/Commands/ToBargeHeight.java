// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LowerJaw;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.UpperJaw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToBargeHeight extends ParallelCommandGroup {
  /** Creates a new ToBargeHeight. */
  public ToBargeHeight(Elevator elevator, Pivot pivot, int slot, UpperJaw upperJaw, LowerJaw lowerJaw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    final double speed_lower_jaw = -.1;
    final double speed_upper_jaw = -.3;
    addCommands(new ElevateNoEnd(elevator, pivot, slot).withTimeout(.5)
    .andThen(
      new ParallelCommandGroup(elevator.c_goToSetPoint(Elevator.Level.LEVEL_4, slot), new IntakeAlgae(upperJaw, lowerJaw, speed_upper_jaw, speed_lower_jaw))));
  }
}
