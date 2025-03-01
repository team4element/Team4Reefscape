// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
public class Climb extends SubsystemBase{
    TalonFX m_climb;
    CurrentLimitsConfigs m_limitConfig = new CurrentLimitsConfigs();
    private DutyCycleOut m_dutyCycle;
    private PositionVoltage m_request;
    public Climb(){
        m_climb = new TalonFX(ClimbConstants.climbID);
        m_dutyCycle = new DutyCycleOut(.5);
        m_request = new PositionVoltage(0);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        config.Slot0.kP = .5;
        TalonFXConfigurator configurator = m_climb.getConfigurator();
        m_climb.getConfigurator().apply(config);
        m_limitConfig.StatorCurrentLimit = ElevatorConstants.statorLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;
        configurator.apply(m_limitConfig);
    }
    public void runMotor(double speed){
        m_climb.setControl(m_dutyCycle.withOutput(speed));
    }
    public void reset(){
        m_climb.setPosition(0);
    }
    public void manualPivot(double speed){
        final double max_speed = .1;
        m_climb.setControl(m_dutyCycle.withOutput(speed * max_speed));
    }
    public void goToSetPoint(double setPoint){
        m_climb.setControl(m_request.withPosition(setPoint));
      }
    @Override
    public void periodic(){
        System.out.println(m_climb.getRotorPosition() + " | " + m_climb.getPosition());
    }
     public Command c_pivotManual(){
    return Commands.run(() -> manualPivot(ControllerConstants.operatorController.getRightY()), this);
  }
}
