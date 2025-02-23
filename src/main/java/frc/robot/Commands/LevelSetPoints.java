package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class LevelSetPoints extends Command {
  Elevator m_elevator = new Elevator();
  double m_setpoint = 0;

  public LevelSetPoints(Elevator elevator, double level) {
    m_elevator = elevator;
    m_setpoint = level;
    addRequirements(elevator);
  }


  @Override
  public void initialize() {
    m_elevator.goToSetPoint(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.setMotors(0);
  }

}