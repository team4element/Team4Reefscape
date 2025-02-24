// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LowerJaw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomaticPivot extends Command {
  /** Creates a new automaticPivot. */

  private LowerJaw m_lowerJaw;

  public AutomaticPivot(LowerJaw lowerJaw) {
    m_lowerJaw = lowerJaw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lowerJaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lowerJaw.automaticPivot();
    m_lowerJaw.innerMotor(-0.25/2);
    m_lowerJaw.outerMotor(-0.25/2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lowerJaw.pivotOff();
    m_lowerJaw.motorsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
