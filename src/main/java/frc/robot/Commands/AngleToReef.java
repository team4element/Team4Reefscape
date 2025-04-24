// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleToReef extends Command {
  /** Creates a new AngleToReef. */
  private Translation2d blue_reef=new Translation2d(4.5, 4.);
  private Translation2d red_reef=new Translation2d(17.55-4.5,4.);
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;
  double m_max_speed;
  PIDController thetaController=new PIDController(0.2, 0, 0);
  SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
  .withDeadband(VisionConstants.deadband*2)
  .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
  .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
  private double HexAngle(){

    Rotation2d angleToReef=drivetrain.getState().Pose.getTranslation().minus(DriverStation.getAlliance().get()==Alliance.Red?red_reef:blue_reef).getAngle();
    double snapped=Rotation2d.fromDegrees(((int)((angleToReef.getDegrees()+(30.*Math.signum(angleToReef.getDegrees())))/60.))*60.).getDegrees()+180;

    return snapped;
  }
  public AngleToReef(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, double max_speed) {
    this.drivetrain=drivetrain;
    this.controller=controller;
    this.m_max_speed=max_speed;
    thetaController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  double targetangle=0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Math.abs(8.775-drivetrain.getState().Pose.getX())<2){
      targetangle=DriverStation.getAlliance().get()==Alliance.Blue?-180:180;
    }else if(ControllerConstants.operatorController.x().getAsBoolean()){
      if(DriverStation.getAlliance().get()==Alliance.Red){
        targetangle=drivetrain.getState().Pose.getY()>4?45:-45;
      }else{
        targetangle=drivetrain.getState().Pose.getY()>4?125:-135;
      }
    }else{
      targetangle=HexAngle();
    }
    thetaController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(
      m_drive
      .withRotationalRate(thetaController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(),targetangle))
      .withVelocityX(-controller.getLeftY() * m_max_speed)
      .withVelocityY(-controller.getLeftX() * m_max_speed));
    //targetangle=HexAngle();
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
