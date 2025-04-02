// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.Pipeline;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class reefHorizontal extends Command {
  /** Creates a new reefHorizontal. */
  Vision m_vision;
  CommandSwerveDrivetrain m_drivetrain;
  private SwerveRequest.RobotCentric m_drive;
     private PIDController m_pid;
      CommandXboxController m_controller;

     double m_maxSpeed;

     double currentPose;
     double targetPose; 

  public reefHorizontal(Vision vision, CommandSwerveDrivetrain drivetrain, double maxSpeed, CommandXboxController controller) {
    m_vision = vision;
    m_drivetrain = drivetrain;
    m_maxSpeed = maxSpeed;
    m_controller = controller; 

     m_pid = new PIDController(0.5, 0, 0);

      m_drive = new SwerveRequest.RobotCentric()
      .withDeadband(VisionConstants.deadband)
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.switchPipeline(Pipeline.LEFT_PIPE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = LimelightHelpers.getTY("");
    targetPose = 0;
    double error = targetPose - currentPose;
    double output = m_pid.calculate(error) * m_maxSpeed * 0.5;

    m_drivetrain.setControl(
      m_drive.withVelocityY(output).withVelocityX(-m_controller.getLeftY() * m_maxSpeed)
      );

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
