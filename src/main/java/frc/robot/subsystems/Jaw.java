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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JawConstants;

public class Jaw extends SubsystemBase {
    private TalonFX m_innerBottomFollower;
    private TalonFX m_outerBottomLeader;
    private TalonFX m_topLeader;
    private TalonFX m_jawPivot;

    private DutyCycleOut m_outerControlRequest;
    private DutyCycleOut m_topControlRequest;

    int m_invert = -1;
    double m_angle;

    TalonFXConfiguration angleConfigs;

    private final PIDController m_pid;

    public MotorOutputConfigs currentConfigs = new MotorOutputConfigs();

     CurrentLimitsConfigs m_limitConfig = new CurrentLimitsConfigs();

    public static enum JawAction{
        INTAKE_ALGAE,
        OUTTAKE_ALGAE,
    }

    public Jaw(){
        TalonFXConfigurator bottomConfigurator = m_innerBottomFollower.getConfigurator();
        TalonFXConfigurator outerConfigurator = m_outerBottomLeader.getConfigurator();

        m_outerBottomLeader = new TalonFX(JawConstants.outerBottomLeaderId);
        m_innerBottomFollower = new TalonFX(JawConstants.innerBottomFollowerId);

        m_topLeader = new TalonFX(JawConstants.TopLeaderId);
        m_innerBottomFollower.setControl(new Follower(JawConstants.outerBottomLeaderId, true));
               
        m_jawPivot = new TalonFX(JawConstants.jawPivotId);  

        m_pid = new PIDController(JawConstants.kP, JawConstants.kI, JawConstants.kD);

        //InnerBottom is going CCW or CW+
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_innerBottomFollower.getConfigurator().apply(currentConfigs);
         //OuterBottom is going CW or CCW+
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_outerBottomLeader.getConfigurator().apply(currentConfigs);

        m_jawPivot.setNeutralMode(NeutralModeValue.Brake);

        m_limitConfig.StatorCurrentLimit = JawConstants.statorLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;

        m_limitConfig.SupplyCurrentLimit = JawConstants.supplyLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;

        //should seperate the config constants later to make it unique to each motor 
        bottomConfigurator.apply(m_limitConfig);
        outerConfigurator.apply(m_limitConfig);
    }


    public void setJaw(double speed){
        m_outerBottomLeader.setControl(m_outerControlRequest.withOutput(speed));
        m_topLeader.setControl(m_topControlRequest.withOutput(speed));
    }
  
 
    public void motorOff(TalonFX motor){
        motor.set(0);
    }

      public void setPID(){

        double p = SmartDashboard.getNumber(JawConstants.tableP, JawConstants.kP);
        double i = SmartDashboard.getNumber(JawConstants.tableI, JawConstants.kI);
        double d = SmartDashboard.getNumber(JawConstants.tableD, JawConstants.kD);
        m_pid.setPID(p, i, d);
      }
    
     //Algae Intake
 public Command c_intakeAlgae(JawAction jawAction, double speed){
    speed = Math.abs(speed);
    double modifiedSpeed = jawAction == JawAction.INTAKE_ALGAE ? speed : -speed;

    return startEnd(() -> setJaw(modifiedSpeed), () -> setJaw(0.5));   
}
   

  public void periodic(){

    setPID();
  }
    
}
