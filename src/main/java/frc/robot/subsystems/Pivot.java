// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JawConstants;
import frc.robot.subsystems.Elevator.Level;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private TalonFX m_jawPivot;
  private TalonFXConfiguration config;
  private PositionVoltage m_request;
  private DutyCycleOut m_duty_cycle;

  
  public Pivot() {
      config = new TalonFXConfiguration();
      m_jawPivot = new TalonFX(JawConstants.jawPivotId);
      m_duty_cycle = new DutyCycleOut(.8);
      m_request = new PositionVoltage(0);

      m_jawPivot.getConfigurator().apply(config);
      config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
      config.Feedback.SensorToMechanismRatio = 12;
      config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      config.Slot0.kP =  .4;
      config.Slot0.kD = .001;
      config.Slot0.kV = .004;
      config.Slot0.kA = .004;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorOff(){
    m_jawPivot.set(0);
    m_jawPivot.setNeutralMode(NeutralModeValue.Brake);     
  }

      //Pivot to move the Coral intake
  public void manualPivot(double speed){
      final double max_speed = .5;
      m_jawPivot.setControl(m_duty_cycle.withOutput(speed * max_speed));
  }

  public Command c_pivotManual(){
    return Commands.run(() -> manualPivot(ControllerConstants.operatorController.getLeftY() * .3), this);
  }

  public void goToSetPoint(double setPoint){
    m_jawPivot.setControl(m_request.withPosition(setPoint));
  }

  public void jawOff(){
    m_jawPivot.set(0);
    m_jawPivot.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command c_goToSetPoint(Level position){
    return startEnd(() -> goToSetPoint(positionToSetpoint(position)), () -> jawOff());
  }

  public void resetPivotEncoder(){
    m_jawPivot.setPosition(0);
  }

  private double positionToSetpoint(Level level){
    switch (level) {
        case LEVEL_1: return .5;
        case LEVEL_2: return -3.5; //Estimate have to test
        case LEVEL_3: return -3.5; //Estimate have to test
        case LEVEL_4: return 0;
        case CORAL_STATION: return -7.45;
    }

    return 0;
  }
}
