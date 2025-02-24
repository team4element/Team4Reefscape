// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Level;
import frc.robot.subsystems.LowerJaw;

public class ElevateAndJaw extends ParallelCommandGroup {
  public ElevateAndJaw(Elevator elevator, LowerJaw lowerJaw, Level elevatorLevel) {
    addCommands(elevator.c_goToSetPoint(elevatorLevel), lowerJaw.c_goToSetPoint(elevatorLevel));
  }
}
