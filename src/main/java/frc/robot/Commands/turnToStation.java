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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class turnToStation extends Command {
  // /** Creates a new turnToStation. */
  // CommandSwerveDrivetrain m_drivetrain;
  // private SwerveRequest.RobotCentric m_drive;
  //   private PIDController m_pid;
  //  double m_maxSpeed;

  // public turnToStation(CommandSwerveDrivetrain drivetrain, double MaxSpeed) {
  //   m_drivetrain = drivetrain;
  //   m_maxSpeed = MaxSpeed;

  //   m_pid = new PIDController(VisionConstants.AutoMove_P, VisionConstants.AutoMove_I, VisionConstants.AutoMove_D);

  //    SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
  //    .withDeadband(VisionConstants.deadband)
  //    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
  //    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   addRequirements(drivetrain);
  // }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   if(m_drivetrain.getState().Pose.getY() > 0.8){
  //   m_drivetrain.setControl(
  //           m_drive
  //             .withRotationalRate(m_pid.calculate(35 * (Math.PI / 180)) * m_maxSpeed));
  //   } else if (m_drivetrain.getState().Pose.getY() < ...){
  //     m_drivetrain.setControl(
  //             m_drive
  //               .withRotationalRate(m_pid.calculate(35 * (Math.PI / 180)) * m_maxSpeed));
  //     }

  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
