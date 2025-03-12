// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ApriltagAllignment extends SequentialCommandGroup {
  /** Creates a new ApriltagAllignment. */
  public ApriltagAllignment(CommandSwerveDrivetrain drivetrain, Vision vision, double speed, CommandXboxController controller, double rotationalSpeed, double translationalSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Shift(drivetrain, vision, controller, rotationalSpeed, translationalSpeed), new ApproachApriltag(drivetrain, vision, speed));
  }
}
