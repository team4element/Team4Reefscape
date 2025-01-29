package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveHorizontal extends Command{    
  private CommandSwerveDrivetrain m_drive_train;
  private Vision m_vision;
  private SwerveRequest.FieldCentric m_drive;

  public MoveHorizontal(CommandSwerveDrivetrain drive_train, Vision vision) {
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
    final double threshold = 1;
    System.out.println("In execite, executing rotational rate " + (m_vision.getHorizontalOffset()));

    if( offset < -threshold){
      m_drive_train.setControl(
        // new SwerveRequest.FieldCentric()
        m_drive.withVelocityY(max_speed)
      );
    }else if (offset > threshold) {
      m_drive_train.setControl(
        // new SwerveRequest.FieldCentric()
        m_drive.withVelocityY(-max_speed)
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
