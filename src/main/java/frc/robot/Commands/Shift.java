// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.Pipeline;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shift extends Command {

  private CommandSwerveDrivetrain m_drive_train;
  private Vision m_vision;
  private SwerveRequest.FieldCentric m_drive;
  private double m_threshold, m_max_speed;
  private boolean m_is_finished;
  private PIDController m_pid;
  private Pipeline m_pipeline;

  public Shift(CommandSwerveDrivetrain drive_train, Vision vision, double max_speed, Pipeline pipeline) {
    m_drive_train = drive_train;
    m_vision = vision;
    m_threshold = 0;
    m_max_speed = max_speed;
    m_pipeline = pipeline;

    m_pid = new PIDController(VisionConstants.Shift_P, VisionConstants.Shift_I, VisionConstants.Shift_D); //TODO better tune

    m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(VisionConstants.deadband)
      .withRotationalDeadband(VisionConstants.rotationalDeadband)
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    addRequirements(drive_train, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.switchPipeline(m_pipeline);
    m_is_finished = false;
    m_threshold = .2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //testing horizontal movement (in progress and might change back to angular movement)
    if (m_vision.hasTarget() && m_threshold < Math.abs(m_vision.getVerticalOffset())) { // + 1.85
        m_drive_train.setControl(
          m_drive.withVelocityY(m_pid.calculate(m_vision.getVerticalOffset() + 1.2 * m_max_speed))); // +2.2
    }else{
        m_is_finished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  m_is_finished;
  }
}
