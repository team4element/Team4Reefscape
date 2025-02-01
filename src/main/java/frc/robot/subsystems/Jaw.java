package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JawConstants;

public class Jaw extends SubsystemBase {
    private TalonFX m_innerBottom;
    private TalonFX m_outerBottom;
    private TalonFX m_jawPivot;

    int m_invert = -1;
    double m_angle;

    TalonFXConfiguration angleConfigs;

    public MotorOutputConfigs currentConfigs = new MotorOutputConfigs();

    public static enum JawAction{
        INTAKE_CORAL,
        OUTTAKE_CORAL,
        INTAKE_ALGAE,
        OUTTAKE_ALGAE
    }

    public Jaw(){

        m_innerBottom = new TalonFX(JawConstants.innerBottomId);
        m_outerBottom = new TalonFX(JawConstants.outerBottomId);

        m_jawPivot = new TalonFX(JawConstants.jawPivotId);  

        //InnerBottom is going CCW or CW+
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
   m_innerBottom.getConfigurator().apply(currentConfigs);
         //OuterBottom is going CW or CCW+
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
   m_outerBottom.getConfigurator().apply(currentConfigs);

    }

    public void controlMotors(JawAction action){
        switch (action) {
            case INTAKE_CORAL: {
                
            } break;
            default:
                break;
        }
    }
 
    public void motorOff(TalonFX motor){
        motor.set(0);
    }

    public void motorOff(WPI_VictorSPX motor){
        motor.set(0);
    }

    public void zeroEncoder(){
        m_jawPivot.setPosition(0, 1);
    }
    

    //Pivot to move the Coral intake
    public void Pivot(double speed){
        if(speed + m_invert > 0 && m_angle >= JawConstants.limitForward
       || speed + m_invert < 0 && m_angle <= JawConstants.limitBackwards) {

        m_jawPivot.set(0);

     } else{
        m_jawPivot.set(speed * m_invert);
        }
    }

    //Assigns a part of the controller to move the pivot
    public Command c_pivotManual(){
        return Commands.run(() -> Pivot(ControllerConstants.operatorController.getLeftY() * .2), this);
    }

    public void motorsOff(){
    m_innerBottom.set(0);
    m_outerBottom.set(0);
  }

    public void motorOff(){
    m_jawPivot.set(0);
}

  public void periodic(){

  }
    
}
