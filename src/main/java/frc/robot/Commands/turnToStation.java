// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.coralStation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class turnToStation extends Command {
   //** Creates a new turnToStation. */
   CommandSwerveDrivetrain m_drivetrain;
   private SwerveRequest.RobotCentric m_drive;
     private PIDController m_pid;
    double m_maxSpeed;

    coralStation m_coralstation;

    double currentPose;
    double targetPose; 

   public turnToStation(CommandSwerveDrivetrain drivetrain, double MaxSpeed) {
     m_drivetrain = drivetrain;
     m_maxSpeed = MaxSpeed;
    //  m_coralstation = coralstation;

     m_pid = new PIDController(0.5, 0, 0);

      m_drive = new SwerveRequest.RobotCentric()
      .withDeadband(VisionConstants.deadband)
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
     // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(drivetrain);
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {

   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
    //currentPose = new Rotation2d(m_drivetrain.getState().Pose.getRotation().getRadians());
    //targetPose = new Rotation2d(m_drivetrain.targetPose(m_coralstation) * (Math.PI/180));
    currentPose = LimelightHelpers.getTargetPose3d_RobotSpace("").getRotation().getY();
    targetPose = 0;
    double error = targetPose - currentPose;
  //  m_pid.enableContinuousInput(-180,180);
  //  double gyroValue = m_drivetrain.getPigeon2().getYaw().getValueAsDouble();
    double output = m_pid.calculate(error) * m_maxSpeed * 1.5;
    System.out.println("current" + m_drivetrain.getState().Pose.getRotation().getDegrees());
    System.out.println("target" + m_drivetrain.targetPose(m_coralstation));
    System.out.println("output"+ output);
   // System.out.println("gyro" + gyroValue);
    System.out.println("error" + error);

     m_drivetrain.setControl(
      m_drive.withRotationalRate((-output))
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
