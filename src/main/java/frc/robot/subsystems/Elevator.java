package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    TalonFX m_rightElevator = new TalonFX(ElevatorConstants.rightElevatorId);
    TalonFX m_leftElevator = new TalonFX(ElevatorConstants.leftElevatorId);
}
