// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Level;
import frc.robot.subsystems.Pivot;

public class ElevateAndPivot extends ParallelCommandGroup {
  public ElevateAndPivot(Elevator elevator, Pivot pivot, Level elevatorLevel, int slot, int slot1) {
    addCommands(elevator.c_goToSetPoint(elevatorLevel, slot), pivot.c_goToSetPoint(elevatorLevel, slot1));
  }
}
