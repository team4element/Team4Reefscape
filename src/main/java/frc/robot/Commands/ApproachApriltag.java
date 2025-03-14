package frc.robot.Commands;


import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.Pipeline;

public class ApproachApriltag extends Command{

    CommandSwerveDrivetrain m_drivetrain;
    Vision m_limelight;

    SwerveRequest.RobotCentric m_drive;
    double m_speed;
    PIDController m_pid;
    boolean m_is_finished;
    DriveRequestType driveRequest;
    Pose2d initialPose;
    Pose2d currentPose;
    double PValue;

    public ApproachApriltag(CommandSwerveDrivetrain drivetrain, Vision limelight, double speed){
      
      m_drivetrain = drivetrain;
      m_limelight = limelight;

      m_speed = speed;

      m_pid = new PIDController(VisionConstants.ApproachApriltag_P, VisionConstants.ApproachApriltag_I, VisionConstants.ApproachApriltag_D); //TODO better tune

        m_drive = new SwerveRequest.RobotCentric()
      .withDeadband(VisionConstants.deadband)
      .withRotationalDeadband(VisionConstants.rotationalDeadband)
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    //   System.out.println(drivetrain);
        addRequirements(drivetrain, limelight);
    }

 // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.switchPipeline(Pipeline.CENTER);
    m_is_finished = false;
    initialPose = m_drivetrain.getState().Pose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = m_drivetrain.getState().Pose;

    //if the limelight doesn't not see a target and the distance formula is positive
    // then have the drivetrain move using the limelight's distance to the apriltag
    double distance = Math.sqrt(Math.pow(currentPose.getX() - initialPose.getX(), 2) + Math.pow(currentPose.getY() - initialPose.getY(), 2));
    double distanceLeftover = (m_limelight.lastKnownTargetDistanceInches / VisionConstants.inchesToMeters) - distance + VisionConstants.inaccuracy;
    if (!m_limelight.hasTarget() && distanceLeftover > 0.02) {
        m_drivetrain.setControl(
          m_drive
          .withVelocityX(-m_pid.calculate(distanceLeftover) * m_speed));
    }else{
        m_is_finished = true;
    }

     System.out.println(distanceLeftover);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_is_finished;
    return false;
  }
    
}
