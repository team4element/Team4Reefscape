// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnInPlace extends Command {
  private Double m_default = VisionConstants.defaultThreshold;

  private GenericEntry turnThreshold = MoveHorizontal.tab.add("Vision Threshold", m_default)
         .getEntry();
  
  private CommandSwerveDrivetrain m_drive_train;
  private Vision m_vision;
  private SwerveRequest.FieldCentric m_drive;

  public TurnInPlace(CommandSwerveDrivetrain drive_train, Vision vision) {
    m_drive_train = drive_train;
    m_vision = vision;
    m_drive = new SwerveRequest.FieldCentric();
    addRequirements(drive_train, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if sees object
    final double max_speed = .5;
    final double offset = m_vision.getHorizontalOffset();
    final double threshold = turnThreshold.getDouble(m_default);
    System.out.println("In execite, executing rotational rate " + (m_vision.getHorizontalOffset()));

    if( offset < -threshold){
      m_drive_train.setControl(
        // new SwerveRequest.FieldCentric()
        m_drive.withRotationalRate(max_speed)
      );
    }else if (offset > threshold) {
      m_drive_train.setControl(
        // new SwerveRequest.FieldCentric()
        m_drive.withRotationalRate(-max_speed)
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
