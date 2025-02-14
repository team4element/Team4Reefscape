package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    TalonFX m_rightFollower;
    TalonFX m_leftLeader;

    public MotorOutputConfigs currentConfigs = new MotorOutputConfigs();

    private PositionDutyCycle m_leftDutyCycle;

    double target = 0;

    PositionVoltage m_request;

    public static enum Levels{
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        LEVEL_4,
        CORAL_STATION
    }

    public Elevator(){    
        m_rightFollower = new TalonFX(ElevatorConstants.rightFollowerId);
        m_leftLeader = new TalonFX(ElevatorConstants.leftLeaderId);

        m_rightFollower.setControl(new Follower(ElevatorConstants.leftLeaderId, true));

        m_request = new PositionVoltage(target).withSlot(0);
        
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_leftLeader.getConfigurator().apply(currentConfigs);
        
        m_rightFollower.setNeutralMode(NeutralModeValue.Brake);
        m_leftLeader.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setMotors(double speed) {
        m_leftLeader.set(speed);        
     }


    public void goToSetPoint(double setPoint){
        target = setPoint;
        m_leftLeader.setControl(m_request.withPosition(setPoint));
    } 

    public double getCurrentPosition() {
        return m_leftLeader.getPosition().getValueAsDouble();
      }

}
