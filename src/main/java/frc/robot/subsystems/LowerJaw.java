package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.JawConstants;

public class LowerJaw extends UpperJaw {
    private TalonFX m_innerBottom;
    private TalonFX m_outerBottom;
    private DutyCycleOut m_outerControlRequest;
    private MotorOutputConfigs currentConfigs;

    public static enum LowerJawAction {
        INTAKE_CORAL,
        OUTTAKE_CORAL,
    }

    public LowerJaw() {
        currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_innerBottom = new TalonFX(JawConstants.innerBottomId);
        m_outerBottom = new TalonFX(JawConstants.outerBottomId);
        m_outerControlRequest = new DutyCycleOut(.5);

        // InnerBottom is going CCW or CW+
        m_innerBottom.getConfigurator().apply(currentConfigs);
        // OuterBottom is going CW or CCW+
        m_outerBottom.getConfigurator().apply(currentConfigs);
    }

    public void setLowerJaw(double outer_speed, double inner_speed) {
        m_outerBottom.setControl(m_outerControlRequest.withOutput(outer_speed));
        m_innerBottom.setControl(m_outerControlRequest.withOutput(inner_speed));
    }

    public void motorsOff() {
        m_innerBottom.set(0);
        m_outerBottom.set(0);
        m_innerBottom.setNeutralMode(NeutralModeValue.Brake);
        m_outerBottom.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command c_intakeCoral(double speed) {
        return startEnd(() -> setLowerJaw(speed, speed), () -> motorsOff());
    }

    @Override
    public void periodic() {

    }

}