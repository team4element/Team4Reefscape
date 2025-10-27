// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class alignToBarge extends Command {
  /** Creates a new alignToBarge. */
  //option 1
//   private Translation2d blue_reef=new Translation2d(4.5, 4.);
 // private Translation2d red_reef=new Translation2d(17.55-4.5,4.);
  CommandSwerveDrivetrain drivetrain;
  double m_max_speed;
  PIDController pid =new PIDController(0.2, 0, 0);
 
  //option 2 + 3
  double goal;
  double current;

  //option 1
 // Translation2d targetDistance = new Translation2d(0,0);

  SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
  .withDeadband(VisionConstants.deadband*2)
  .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
  .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

  public alignToBarge(CommandSwerveDrivetrain drivetrain, double max_speed) {
    this.drivetrain=drivetrain;
    this.m_max_speed=max_speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    goal = LimelightHelpers.getTY("");

    //optional 2
    // goal = LimelightHelpers.getTargetPose3d_RobotSpace("").getY();
    // current = drivetrain.getState().Pose.getY();

    //option 1
    // if(Math.abs(8.775-drivetrain.getState().Pose.getX())<=3 && !LimelightHelpers.getTV("")){
    //   if(DriverStation.getAlliance().get() == Alliance.Blue){
    //   targetDistance = new Translation2d(drivetrain.getState().Pose.getX(),2);
    //   } else if (DriverStation.getAlliance().get() == Alliance.Red){
    //     targetDistance = new Translation2d(drivetrain.getState().Pose.getX(),2-4);
    //   }
    // }
    // pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //option 1
   // drivetrain.setControl(
      // m_drive
      // .withVelocityX(-ControllerConstants.driverController.getLeftY() * m_max_speed)
      // .withVelocityY(pid.calculate(targetDistance.getY() * m_max_speed)));

    //option 2
      // drivetrain.setControl(
      //   m_drive
      //   .withVelocityX(-ControllerConstants.driverController.getLeftY() * m_max_speed)
      //   .withVelocityY(pid.calculate((goal - current) * m_max_speed * 0.1))
      // );
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
