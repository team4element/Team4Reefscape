// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShuffleboardHelper;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoMove extends Command {
  private double m_rotation_speed;
  private double m_vertical_speed;
  private double m_horizontal_speed;
  private CommandSwerveDrivetrain.AutoMoveAction m_action;

  private CommandSwerveDrivetrain m_drive_train;
  private Vision m_vision;
  private SwerveRequest.FieldCentric m_drive;
  private double m_threshold;
  private boolean m_is_finished;
  private PIDController m_pid;

  public AutoMove(CommandSwerveDrivetrain drive_train, Vision vision, CommandSwerveDrivetrain.AutoMoveAction action) {
    m_drive_train = drive_train;
    m_vision = vision;
    m_rotation_speed = m_horizontal_speed = m_vertical_speed = 0;
    m_threshold = 0;
    m_action = action;

    m_pid = new PIDController(.12, 0, 0.003);

    m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(.3)
      .withRotationalDeadband(.1)
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    addRequirements(drive_train, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_is_finished = false;
      switch (m_action) {
        case TURN_IN_PLACE:
          m_rotation_speed = .5;
          m_vision.switchPipeline(Vision.Pipeline.TWO_DIMENSIONAL);
          m_threshold = ShuffleboardHelper.getInstance().getTurnThreshold();
          m_threshold = .5;
          break;
        case MOVE_HORIZONTAL:
          m_vision.switchPipeline(Vision.Pipeline.THREE_DIMENSIONAL);
          m_horizontal_speed = 1.5;
          m_threshold = ShuffleboardHelper.getInstance().getHorizontalThreshold();
          break;
        case MOVE_VERTICAL:
          m_vision.switchPipeline(Vision.Pipeline.THREE_DIMENSIONAL);
          m_vertical_speed = -1.5;
          m_threshold = 2.0;
          break;
        default: break;
      }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offset = 0;

    switch (m_action) {
      case MOVE_HORIZONTAL: //fallthrough
      case TURN_IN_PLACE: offset = m_vision.getHorizontalOffset(); break;
      case MOVE_VERTICAL: offset = m_vision.getTarget3DPose().getZ();
        // System.out.printf("\r\n X: %f, Y: %f, Z: %f", m_vision.getTarget3DPose().getX(), m_vision.getTarget3DPose().getY(), m_vision.getTarget3DPose().getZ());
      break;
      default: break;
    }

    System.out.printf("\r\n threshold %f | offset: %f", m_threshold, offset);

    if (m_vision.hasTarget()) {
      if (offset < -m_threshold) {
        m_drive_train.setControl(
            m_drive
              .withRotationalRate(m_pid.calculate(m_vision.getHorizontalOffset()) * m_rotation_speed)
              .withVelocityX(m_vertical_speed)
              .withVelocityY(m_horizontal_speed));
      } else if (offset > m_threshold) {
        m_drive_train.setControl(
          m_drive
          .withRotationalRate(m_pid.calculate(m_vision.getHorizontalOffset()) * m_rotation_speed)
          .withVelocityX(-m_vertical_speed)
          .withVelocityY(-m_horizontal_speed));
      }else{
        m_is_finished = true;
      }
    }else{
      m_is_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_is_finished;
  }
}