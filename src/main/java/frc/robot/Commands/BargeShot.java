// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.JawConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LowerJaw;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.UpperJaw;

/** Add your docs here. */
public class BargeShot extends ParallelCommandGroup{

    public BargeShot( Pivot pivot, UpperJaw upperjaw, LowerJaw lowerjaw){
        final double outtake_speed_lower_jaw = -.1;
        final double outtake_speed_upper_jaw = -.3;
        addCommands(new IntakeAlgaeNoEnd(upperjaw, lowerjaw, outtake_speed_lower_jaw, outtake_speed_upper_jaw)
        .withTimeout(0.065).andThen( new ParallelCommandGroup 
        (new IntakeAlgae(upperjaw, lowerjaw, outtake_speed_lower_jaw, outtake_speed_upper_jaw),
         pivot.c_goToSetPoint(Elevator.Level.LEVEL_4, 1)).withTimeout(5))
        );
    }
}
