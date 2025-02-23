package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JawConstants;

public class Jaw extends SubsystemBase {
    private TalonFX m_innerBottom;
    private TalonFX m_outerBottom;
    private TalonFX m_top;
    private TalonFX m_jawPivot;

    private DutyCycleOut m_outerControlRequest;
    private DutyCycleOut m_topControlRequest;
    private DutyCycleOut m_innerControlRequest;

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

    public Jaw(){
        m_outerBottom = new TalonFX(JawConstants.outerBottomId);
        m_innerBottom = new TalonFX(JawConstants.innerBottomId);
        m_jawPivot = new TalonFX(JawConstants.jawPivotId); 
        m_top = new TalonFX(JawConstants.TopId);
        
        m_outerControlRequest = new DutyCycleOut(.5);
        m_topControlRequest = new DutyCycleOut(0.5);
        m_innerControlRequest = new DutyCycleOut(0.5);

        currentConfigs = new MotorOutputConfigs();
        m_limitConfig = new CurrentLimitsConfigs();

        TalonFXConfigurator bottomConfigurator = m_innerBottom.getConfigurator();
        TalonFXConfigurator outerConfigurator = m_outerBottom.getConfigurator();

        TalonFXConfigurator jawConfigurator = m_jawPivot.getConfigurator();

        m_pid = new PIDController(JawConstants.kP, JawConstants.kI, JawConstants.kD);

        //InnerBottom is going CCW+
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_innerBottom.getConfigurator().apply(currentConfigs);
         //OuterBottom is going CW
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_outerBottom.getConfigurator().apply(currentConfigs);
        //Top motor is going CW
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_top.getConfigurator().apply(currentConfigs);

        m_limitConfig.StatorCurrentLimit = JawConstants.statorLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;

        m_limitConfig.SupplyCurrentLimit = JawConstants.supplyLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;

        //should seperate the config constants later to make it unique to each motor 
        bottomConfigurator.apply(m_limitConfig);
        outerConfigurator.apply(m_limitConfig);
        jawConfigurator.apply(m_limitConfig);
    }


    public void setJaw(double speed){
        m_outerBottom.setControl(m_outerControlRequest.withOutput(-speed));
        m_innerBottom.setControl(m_innerControlRequest.withOutput(speed));
        m_top.setControl(m_topControlRequest.withOutput(speed));
    }
  
 
    public void motorsOff(){
        m_outerBottom.set(0);
        m_innerBottom.set(0);
        m_top.set(0);

        m_top.setNeutralMode(NeutralModeValue.Brake);
        m_innerBottom.setNeutralMode(NeutralModeValue.Brake);
        m_outerBottom.setNeutralMode(NeutralModeValue.Brake);
        m_jawPivot.setNeutralMode(NeutralModeValue.Brake);
    }

      public void setPID(){

        double p = SmartDashboard.getNumber(JawConstants.tableP, JawConstants.kP);
        double i = SmartDashboard.getNumber(JawConstants.tableI, JawConstants.kI);
        double d = SmartDashboard.getNumber(JawConstants.tableD, JawConstants.kD);
        m_pid.setPID(p, i, d);
      }
    
     //Algae Intake
 public Command c_intakeAlgae(double speed){
    return startEnd(() -> setJaw(speed), () -> motorsOff());   
}

  public void periodic(){
    setPID();
  }
    
}
