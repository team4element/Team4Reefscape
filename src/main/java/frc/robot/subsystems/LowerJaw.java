package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JawConstants;
import frc.robot.subsystems.Elevator.Level;

public class LowerJaw extends UpperJaw {

    private TalonFX m_innerBottom;
    private TalonFX m_outerBottom;
   
    private TalonFX m_jawPivot;
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
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        config.Feedback.SensorToMechanismRatio = 12;
        config.Slot0.kP =  .01;
        config.Slot0.kD = .001;
        config.Slot0.kV = .005;
        config.Slot0.kA = .005;
        
        m_innerBottom = new TalonFX(JawConstants.innerBottomId);
        m_outerBottom = new TalonFX(JawConstants.outerBottomId);
        
        m_jawPivot = new TalonFX(JawConstants.jawPivotId);
        m_jawPivot.getConfigurator().apply(config);

        m_outerControlRequest = new DutyCycleOut(.5);
        m_innerControlRequest = new DutyCycleOut(.5);
        m_topControlRequest = new DutyCycleOut(.5);
        
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

    public void motorsOff(){
        m_innerBottom.set(0);
        m_outerBottom.set(0);
        m_jawPivot.setNeutralMode(NeutralModeValue.Brake);     
    }

    //Pivot to move the Coral intake
    public void Pivot(double speed){
        final double max_speed = .5;
        // System.out.println(speed);
        m_jawPivot.setControl(m_topControlRequest.withOutput(speed * max_speed));
    }

  //Commands
  //Assigns a part of the controller to move the pivot
  public Command c_pivotManual(){
    return Commands.run(() -> Pivot(ControllerConstants.operatorController.getLeftY() * .3), this);
}

 public Command c_intakeCoral(double speed){
    return startEnd(() -> setLowerJaw(speed), () -> motorsOff());   
}

public void goToSetPoint(double setPoint){
    m_jawPivot.setControl(m_request.withPosition(setPoint));
}
    
@Override
public void periodic() {
    super.periodic();

    System.out.printf("%s\rn", m_jawPivot.getRotorPosition().toString());
}

private double positionToSetpoint(Level level){
    switch (level) {
        case LEVEL_1: return .5;
        case LEVEL_2: return 0;
        case LEVEL_3: return 0;
        case LEVEL_4: return 0;
        case CORAL_STATION: return 0;
    }

    return 0;
}

public void jawOff(){
    m_jawPivot.set(0);
}

public Command c_goToSetPoint(Level position){
    return startEnd(() -> goToSetPoint(positionToSetpoint(position)), () -> jawOff());
}

}