// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldAngle extends Command {

  private CommandSwerveDrivetrain m_drive_train;
  private Vision m_vision;
  private SwerveRequest.FieldCentric m_drive;
  private double m_threshold, m_max_speed;
  private boolean m_is_finished;
  private PIDController m_pid;
  private CommandXboxController m_controller;
  private double m_max_angle_rate;
  private VelocityVoltage m_requestLeft;


  public HoldAngle(CommandSwerveDrivetrain drive_train, Vision vision, CommandXboxController controller, double max_speed, double max_angle_rate) {
    m_drive_train = drive_train;
    m_vision = vision;
    m_controller = controller;
    m_threshold = 0;
    m_max_speed = max_speed;
    m_max_angle_rate = max_angle_rate;

    m_pid = new PIDController(VisionConstants.HoldAngle_P, VisionConstants.HoldAngle_I, VisionConstants.HoldAngle_D); //TODO better tune

    m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(m_max_speed * VisionConstants.deadband)
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    addRequirements(drive_train, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_is_finished = false;
      m_vision.switchPipeline(Vision.Pipeline.TWO_DIMENSIONAL);
      m_threshold = .2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offset = 0;

    // System.out.printf("\r\n threshold %f | offset: %f", m_threshold, offset);

    //testing horizontal movement (in progress and might change back to angular movement)
    if (m_vision.hasTarget()) {
        m_drive_train.setControl(
          m_drive
          .withRotationalRate(m_pid.calculate(m_vision.getVerticalOffset()) * m_max_angle_rate)
          .withVelocityX(-m_controller.getLeftY() * m_max_speed)
          .withVelocityY(-m_controller.getLeftX() * m_max_speed));
        // System.out.printf("vr: %f  vX: %f  vY: %f\n", m_pid.calculate(m_vision.getVerticalOffset()) * m_max_angle_rate, -m_controller.getLeftY() * m_max_speed, -m_controller.getLeftX() * m_max_speed);
        // System.out.printf("cX: %f  cY: %f\n", m_controller.getLeftX(), m_controller.getLeftY());
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
