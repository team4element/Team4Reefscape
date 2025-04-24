// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class recordValues extends Command {
  /** Creates a new recordValues. */
  Elevator m_elevator;
  Pivot m_pivot;

  public recordValues(Elevator elevator, Pivot pivot) {
    m_elevator = elevator;
    m_pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.recordValue();
    m_pivot.recordValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
