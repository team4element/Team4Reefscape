package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JawConstants;

public class LowerJaw extends Jaw {
    private TalonFX m_innerBottom;
    private TalonFX m_outerBottom;
   
    public TalonFX m_jawPivot;

    private DutyCycleOut m_outerControlRequest;
    private DutyCycleOut m_innerControlRequest;
    private DutyCycleOut m_topControlRequest;
    private DutyCycleOut m_jawControlRequest;

    private final PIDController m_pid;

    int m_invert = -1;

    TalonFXConfiguration angleConfigs;
    public MotorOutputConfigs currentConfigs = new MotorOutputConfigs();

    public static enum LowerJawAction{
        INTAKE_CORAL,
        OUTTAKE_CORAL,        
    }

    public LowerJaw(){
        m_innerBottom = new TalonFX(JawConstants.innerBottomId);
        m_outerBottom = new TalonFX(JawConstants.outerBottomId);
        
        m_jawPivot = new TalonFX(JawConstants.jawPivotId);  
        m_outerControlRequest = new DutyCycleOut(1);
        m_innerControlRequest = new DutyCycleOut(1);
        m_topControlRequest = new DutyCycleOut(1);
        m_jawControlRequest = new DutyCycleOut(1);

        m_pid = new PIDController(JawConstants.kP, JawConstants.kI, JawConstants.kD);
        
        TalonFXConfigurator innerConfigurator = m_innerBottom.getConfigurator();
        TalonFXConfigurator pivotConfigurator = m_jawPivot.getConfigurator();
        
        //InnerBottom is going CCW or CW+
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_innerBottom.getConfigurator().apply(currentConfigs);
         //OuterBottom is going CW or CCW+
      
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_outerBottom.getConfigurator().apply(currentConfigs);
        
        m_limitConfig.StatorCurrentLimit = JawConstants.lowerStatorLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;

        m_limitConfig.SupplyCurrentLimit = JawConstants.lowerSupplyLimit;
        m_limitConfig.SupplyCurrentLimitEnable = true;

        //should seperate the config constants later to make it unique to each motor 
        innerConfigurator.apply(m_limitConfig);
        pivotConfigurator.apply(m_limitConfig);
    }
    
    public void setLowerJaw(double speed){
        m_outerBottom.setControl(m_outerControlRequest.withOutput(speed));
        m_innerBottom.setControl(m_outerControlRequest.withOutput(speed));
    }

    public void setInner(){
        m_outerBottom.setControl(m_outerControlRequest.withOutput(1));
        m_innerBottom.setControl(m_outerControlRequest.withOutput(0.25));
    }

    public void innerMotor(double speed){
        m_innerBottom.setControl(m_outerControlRequest.withOutput(speed));
    }
    
    public void outerMotor(double speed){
        m_outerBottom.setControl(m_outerControlRequest.withOutput(speed));
    }

    public void motorsOff(){
        m_innerBottom.set(0);
        m_outerBottom.set(0);
        m_jawPivot.setNeutralMode(NeutralModeValue.Brake);     
    }

    //Pivot to move the Coral intake
    public void Pivot(double speed){
        if (speed > JawConstants.pivotTolerance || speed < -JawConstants.pivotTolerance){
            final double max_speed = .1;
            m_jawPivot.setControl(m_jawControlRequest.withOutput(speed * max_speed));
        } else {
            m_jawPivot.setControl(m_jawControlRequest.withOutput(0));
        }
    }

    public void automaticPivot(){
        //m_jawPivot.getConfigurator().setPosition(5);
        if (m_jawPivot.getPosition().getValueAsDouble() > JawConstants.limitBackwards && m_jawPivot.getPosition().getValueAsDouble() < JawConstants.limitForward){

        }
        m_jawPivot.setControl(m_jawControlRequest.withOutput(m_pid.calculate(m_jawPivot.get(), -1)));
    }

    public void pivotOff(){
        m_jawPivot.set(0);
    }

  public void periodic(){
        System.out.println(m_jawPivot.getPosition().getValueAsDouble()); //This is to find the encoder value for code. 
        setPID();
  }

  //Commands
  //Assigns a part of the controller to move the pivot
  public Command c_pivotManual(){
    return Commands.run(() -> Pivot(ControllerConstants.operatorController.getLeftY()), this);
}

 public Command c_automaticPivot(){
    return startEnd(() -> automaticPivot(), () -> pivotOff());
 }

 public Command c_intakeCoral(double speed){
    return startEnd(() -> setLowerJaw(speed), () -> motorsOff());   
}

public Command c_outerMotor(){
    return startEnd(() -> m_outerBottom.setControl(m_outerControlRequest.withOutput(1)), () -> m_outerBottom.set(0));
}

public Command c_innerMotor(double speed){
    return startEnd(() -> m_innerBottom.setControl(m_outerControlRequest.withOutput(speed)), () -> m_innerBottom.set(0));
}
    
}
