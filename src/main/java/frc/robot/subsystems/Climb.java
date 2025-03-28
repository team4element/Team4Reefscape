// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
/** Add your docs here. */
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;

public class Climb extends SubsystemBase{
    TalonFX m_right;
    TalonFX m_left;
    CurrentLimitsConfigs m_limitConfig = new CurrentLimitsConfigs();
    private DutyCycleOut m_dutyCycle;
    private PositionVoltage m_request;
    private TalonFXConfiguration config;
    private double m_hold_value;

    //2.7 is max rotation or else brake battery

    public Climb(){
        m_right = new TalonFX(ClimbConstants.climbID);
        m_left = new TalonFX(ClimbConstants.leftID);
        m_dutyCycle = new DutyCycleOut(.5);
        m_request = new PositionVoltage(0);
        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXConfiguration config2 = new TalonFXConfiguration();
        config2.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config2.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        config2.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kP = 1;
        config2.Slot0.kP = 1;

        TalonFXConfigurator configurator = m_right.getConfigurator();
        m_right.getConfigurator().apply(config);
        m_left.getConfigurator().apply(config2);
        m_limitConfig.StatorCurrentLimit = ElevatorConstants.statorLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;
        configurator.apply(m_limitConfig);
    }
    public void runMotorUp(double speed){
       if (m_right.getPosition().getValueAsDouble() >= 70){
           m_right.setControl(m_dutyCycle.withOutput(0));
           m_left.setControl(m_dutyCycle.withOutput(0));
           m_right.setNeutralMode(NeutralModeValue.Brake);
           m_left.setNeutralMode(NeutralModeValue.Brake);
       } else {
            m_right.setControl(m_dutyCycle.withOutput(speed));
            m_left.setControl(m_dutyCycle.withOutput(speed));
       }
    }

    public void runMotorDown(double speed){
       if(m_right.getPosition().getValueAsDouble() <= -37.2) {
           m_right.setControl(m_dutyCycle.withOutput(0));
           m_left.setControl(m_dutyCycle.withOutput(0));
        } else {
            m_right.setControl(m_dutyCycle.withOutput(-speed));
            m_left.setControl(m_dutyCycle.withOutput(-speed));
        }
    }

    public void motorOff(){
        m_right.setControl(m_dutyCycle.withOutput(0));
        m_left.setControl(m_dutyCycle.withOutput(0));
        m_right.setNeutralMode(NeutralModeValue.Brake);
           m_left.setNeutralMode(NeutralModeValue.Brake);
    }
    public void manualPivot(double speed){
        final double max_speed = .1;
        m_right.setControl(m_dutyCycle.withOutput(speed * max_speed));
        m_left.setControl(m_dutyCycle.withOutput(speed * max_speed));
    }

    public void goToSetPoint(double setPoint, double goal){
        m_right.setControl(m_request.withPosition(setPoint));
        m_left.setControl(m_request.withPosition(goal));
      }

    public void reset(){
        m_right.setPosition(0);
        m_left.setPosition(0);
    }

    @Override
    public void periodic(){
       // System.out.println(m_climb.getRotorPosition() + " | " + m_climb.getPosition());
    }
    
    public Command c_pivotPositive(double speed){
        return Commands.startEnd(() -> runMotorUp(speed), () -> motorOff());
  }

  public Command c_pivotNegative(double speed){
    return Commands.startEnd(() -> runMotorDown(speed), () -> motorOff());
}
  public Command c_horizontal(double setpoint, double goal){
    return Commands.startEnd(() -> goToSetPoint(setpoint, goal), () -> motorOff());
  }

}
