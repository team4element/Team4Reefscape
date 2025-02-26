package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JawConstants;
import frc.robot.subsystems.Elevator.Level;

public class LowerJaw extends UpperJaw {

    private TalonFX m_innerBottom;
    private TalonFX m_outerBottom;
   
    private PositionVoltage m_request;

    private DutyCycleOut m_outerControlRequest;
    private DutyCycleOut m_innerControlRequest;
    private DutyCycleOut m_topControlRequest;

    int m_invert = -1;

    TalonFXConfiguration angleConfigs;

    public MotorOutputConfigs currentConfigs = new MotorOutputConfigs();

    public static enum LowerJawAction{
        INTAKE_CORAL,
        OUTTAKE_CORAL,        
    }

    public LowerJaw(){
        m_request = new PositionVoltage(0).withSlot(0);
                
        m_innerBottom = new TalonFX(JawConstants.innerBottomId);
        m_outerBottom = new TalonFX(JawConstants.outerBottomId);
        
        m_outerControlRequest = new DutyCycleOut(.5);
        m_innerControlRequest = new DutyCycleOut(.5);
        m_topControlRequest = new DutyCycleOut(.5);
        
        TalonFXConfigurator innerConfigurator = m_innerBottom.getConfigurator();
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
    }
    
    public void setLowerJaw(double outer_speed, double inner_speed){
        m_outerBottom.setControl(m_outerControlRequest.withOutput(outer_speed));
        m_innerBottom.setControl(m_outerControlRequest.withOutput(inner_speed));
    }

    public void motorsOff(){
        m_innerBottom.set(0);
        m_outerBottom.set(0);
        m_innerBottom.setNeutralMode(NeutralModeValue.Brake);
        m_outerBottom.setNeutralMode(NeutralModeValue.Brake); 
    }

  //Commands
  //Assigns a part of the controller to move the pivot


 public Command c_intakeCoral(double speed){
    return startEnd(() -> setLowerJaw(speed, speed), () -> motorsOff());   
}
    
@Override
public void periodic() {
    super.periodic();
}

}