package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JawConstants;

public class UpperJaw extends SubsystemBase {
    private TalonFX m_top;

    private DutyCycleOut m_topControlRequest;

    int m_invert = -1;
    double m_angle;

    TalonFXConfiguration angleConfigs;

    private final PIDController m_pid;

    public MotorOutputConfigs currentConfigs;
     CurrentLimitsConfigs m_limitConfig;

    public static enum JawAction{
        INTAKE_ALGAE,
        OUTTAKE_ALGAE,
    }

    public UpperJaw(){
        m_top = new TalonFX(JawConstants.TopId);
        
        m_topControlRequest = new DutyCycleOut(0.5);

        currentConfigs = new MotorOutputConfigs();
        m_limitConfig = new CurrentLimitsConfigs();

        m_pid = new PIDController(JawConstants.kP, JawConstants.kI, JawConstants.kD);

        //Top motor is going CW
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_top.getConfigurator().apply(currentConfigs);

        m_limitConfig.StatorCurrentLimit = JawConstants.statorLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;

        m_limitConfig.SupplyCurrentLimit = JawConstants.supplyLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;
    }

    public void setJaw(double speed){
        m_top.setControl(m_topControlRequest.withOutput(speed));
    }
  
    public void motorOff(){
        m_top.set(0);
        m_top.setNeutralMode(NeutralModeValue.Brake);
    }

      public void setPID(){

        double p = SmartDashboard.getNumber(JawConstants.tableP, JawConstants.kP);
        double i = SmartDashboard.getNumber(JawConstants.tableI, JawConstants.kI);
        double d = SmartDashboard.getNumber(JawConstants.tableD, JawConstants.kD);
        m_pid.setPID(p, i, d);
      }
    
  public void periodic(){
    setPID();
  } 
}
