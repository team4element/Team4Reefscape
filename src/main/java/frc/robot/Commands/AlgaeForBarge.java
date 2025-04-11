// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LowerJaw;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.UpperJaw;

/** Add your docs here. */
public class AlgaeForBarge extends ParallelCommandGroup{

    public AlgaeForBarge(UpperJaw upperjaw, LowerJaw lowerjaw, Pivot pivot){
        final double intake_speed_lower_jaw = 0.3;
        final double intake_speed_upper_jaw = 0.3;
        addCommands(new IntakeAlgae(upperjaw, lowerjaw, intake_speed_lower_jaw, intake_speed_upper_jaw), pivot.c_goToSetPoint(Elevator.Level.ALGAE, 1));
    }
}
