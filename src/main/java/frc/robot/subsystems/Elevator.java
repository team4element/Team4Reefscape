package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.HoldAngle;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private TalonFX m_rightFollower;
    private TalonFX m_leftLeader;
    private DutyCycleOut m_leftDutyCycle;
    private PositionVoltage m_request;
    private TalonFXConfiguration config;
    private CurrentLimitsConfigs m_limitConfig;
    private TalonFXConfigurator m_leftConfigurator;
    private TalonFXConfigurator m_rightConfigurator;
    private double m_hold_value;

    // The different levels the elevator needs a setpoint for
    public static enum Level {
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        LEVEL_4,
        CORAL_STATION,
        ALGAE
    }

    public Elevator() {
        m_hold_value = 0;

        config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        config.Feedback.SensorToMechanismRatio = 9;

        //Slot0 is the origial/faster version of PID
        config.Slot0.kP = 11;
        config.Slot0.kD = .1;
        config.Slot0.kV = .005;
        config.Slot0.kA = .005;

        //Slot1 is a slower version of PID
        config.Slot1.kP = 3;
        config.Slot1.kD = .1;
        config.Slot1.kV = .005;
        config.Slot1.kA = .005;

        m_rightFollower = new TalonFX(ElevatorConstants.rightFollowerId);
        m_leftLeader = new TalonFX(ElevatorConstants.leftLeaderId);

        m_request = new PositionVoltage(0).withSlot(0);
        m_limitConfig = new CurrentLimitsConfigs();

        m_leftDutyCycle = new DutyCycleOut(1);

        m_leftConfigurator = m_leftLeader.getConfigurator();
        m_rightConfigurator = m_rightFollower.getConfigurator();

        m_rightFollower.setControl(new Follower(ElevatorConstants.leftLeaderId, true));

        m_leftLeader.getConfigurator().apply(config);
        m_rightFollower.getConfigurator().apply(config);

        m_limitConfig.StatorCurrentLimit = ElevatorConstants.statorLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;

        m_limitConfig.SupplyCurrentLimit = ElevatorConstants.supplyLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;

        // make the current limit seperately later on
        m_leftConfigurator.apply(m_limitConfig);
        m_rightConfigurator.apply(m_limitConfig);

        // This sets the motor to rotate counterclockwise
        m_leftLeader.getConfigurator().apply(config);

        SmartDashboard.putNumber(ElevatorConstants.tableP, ElevatorConstants.kP);
        SmartDashboard.putNumber(ElevatorConstants.tableI, ElevatorConstants.kI);
        SmartDashboard.putNumber(ElevatorConstants.tableD, ElevatorConstants.kD);
    }

    public void setMotors(double speed) {
        if(my_deadband(speed) != 0){
            m_leftLeader.setControl(m_leftDutyCycle.withOutput(speed));
            m_hold_value = m_leftLeader.getPosition().getValueAsDouble();
        }else{
            goToSetPoint(m_hold_value, 0);
        }
    }

    public void motorOff(TalonFX motor) {
        motor.set(0);
        motor.setNeutralMode(NeutralModeValue.Brake);
        m_hold_value = m_leftLeader.getPosition().getValueAsDouble();
    }

    public void holdEnd() {
        m_leftLeader.set(0);
        m_leftLeader.setNeutralMode(NeutralModeValue.Brake);
    }

    public void goToSetPoint(double setPoint, int slot) {
        // If we need to travel dowards then use smalled PID values
        // if (setPoint < getCurrentPosition()) {
            m_leftLeader.setControl(m_request.withPosition(setPoint).withSlot(slot));
        // } else {
            // m_leftLeader.setControl(m_request.withPosition(setPoint).withSlot(0));
        // }
    }

    public Command c_goToSetPoint(Level level, int slot) {
        return startEnd(() -> goToSetPoint(levelToSetPoint(level), slot), () -> motorOff(m_leftLeader));
    }

    public Command c_lower() {
        return startEnd(() -> goToSetPoint(levelToSetPoint(Level.LEVEL_1), 1), () -> motorOff(m_leftLeader));
    }

    public double getCurrentPosition() {
        return m_leftLeader.getPosition().getValueAsDouble();
    }

    public double my_deadband(double input){
        final double deadband = .2;
        return Math.abs(input) > deadband ? input : 0; 
    }

    public Command c_moveElevator() {
        return Commands.run(() -> setMotors(my_deadband(-ControllerConstants.operatorController.getRightY()) * .5), this);
    }

    @Override
    public void periodic() {

    }

    public double goToLevel(Level level){
        switch(level){
            case LEVEL_1: return 2.3;
            case LEVEL_2: return 3.6;
            case LEVEL_3: return 5.6;
            case LEVEL_4: return 7.3;
            case CORAL_STATION: return 3.7;
        }

        return 3;
    }

    public void resetEncoders() {
        m_leftLeader.setPosition(0);
        m_rightFollower.setPosition(0);
    }

    public Command c_hold() {
        // If our elevator is below LEVEL_1 don't continue to run the motor
        if (m_hold_value > levelToSetPoint(Level.LEVEL_1)) {
            return startEnd(() -> goToSetPoint(m_hold_value, 0), () -> holdEnd());
        }
        return startEnd( ()-> holdEnd(), () -> holdEnd());
    }

    public double levelToSetPoint(Level level) {
        switch (level) {
            case LEVEL_1:
                return 2.3;
            case LEVEL_2:
                return 3.6;
            case LEVEL_3:
                return 5.6;
            case LEVEL_4:
                return 7.3;
            case CORAL_STATION:
                return 3.5;
        }
        return 3;
    }
}
