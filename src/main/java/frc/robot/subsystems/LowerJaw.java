package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JawConstants;

public class LowerJaw extends Jaw {
    private TalonFX m_innerBottomFollower;
    private TalonFX m_outerBottomLeader;
   
    private TalonFX m_jawPivot;

    private DutyCycleOut m_outerControlRequest;

    int m_invert = -1;
    double m_angle;

    TalonFXConfiguration angleConfigs;

    public MotorOutputConfigs currentConfigs = new MotorOutputConfigs();

    public static enum JawAction{
        INTAKE_CORAL,
        OUTTAKE_CORAL,        
    }

    public LowerJaw(){

        m_innerBottomFollower = new TalonFX(JawConstants.innerBottomFollowerId);
        m_outerBottomLeader = new TalonFX(JawConstants.outerBottomLeaderId);
        m_innerBottomFollower.setControl(new Follower(JawConstants.outerBottomLeaderId, false));
    
        m_jawPivot = new TalonFX(JawConstants.jawPivotId);  

        //InnerBottom is going CCW or CW+
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
   m_innerBottomFollower.getConfigurator().apply(currentConfigs);
         //OuterBottom is going CW or CCW+
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
   m_outerBottomLeader.getConfigurator().apply(currentConfigs);

        m_jawPivot.setNeutralMode(NeutralModeValue.Brake);        
    }
    
    public void setLowerJaw(double speed){
        m_outerBottomLeader.setControl(m_outerControlRequest.withOutput(speed));
    }

    public void controlMotors(JawAction action){
        switch (action) {
            case INTAKE_CORAL: 
                
                break;
            case OUTTAKE_CORAL:
                break;            
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

    //Pivot to move the Coral intake
    public void Pivot(double speed){
        if(speed + m_invert > 0 && m_angle >= JawConstants.limitForward
       || speed + m_invert < 0 && m_angle <= JawConstants.limitBackwards) {

        m_jawPivot.set(0);

     } else{
        m_jawPivot.set(speed * m_invert);
        }
    }

  public void periodic(){

  }

  //Commands
  //Assigns a part of the controller to move the pivot
  public Command c_pivotManual(){
    return Commands.run(() -> Pivot(ControllerConstants.operatorController.getLeftY() * .2), this);
}

 //TODO: This work??????
 public Command c_intakeCoral(JawAction jawAction, double speed){
    speed = Math.abs(speed);
    double modifiedSpeed = jawAction == JawAction.INTAKE_CORAL ? speed : -speed;

    return startEnd(() -> setLowerJaw(modifiedSpeed), () -> setLowerJaw(0.55));   
}

    
}
