// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LowerJaw;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.UpperJaw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MovingToBarge extends ParallelCommandGroup {
  /** Creates a new MovingToBarge. */
  public MovingToBarge(Elevator elevator, Pivot pivot, UpperJaw upperjaw, LowerJaw lowerjaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(elevator.c_goToSetPoint(Elevator.Level.LEVEL_4, 0), new BargeShot(pivot, upperjaw, lowerjaw));
  }

  // Called when the command is initially scheduled.
}