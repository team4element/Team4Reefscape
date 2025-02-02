package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShuffleboardHelper {
    private static ShuffleboardHelper instance;
    public static ShuffleboardHelper getInstance() {
        if (instance == null) {
            instance = new ShuffleboardHelper();
        }
        return instance;
    }

    private GenericEntry m_turn_threshold;
    private GenericEntry m_horizontal_threshold;

    public void initialize() {
        m_turn_threshold = Shuffleboard.getTab("Vision").add("Turn Threshold", 5.).getEntry();
        m_horizontal_threshold = Shuffleboard.getTab("Vision").add("Horizontal Threshold", 5).getEntry();;
    }

    public double getTurnThreshold() {
        return m_turn_threshold.getDouble(5.);
    }

    public double getHorizontalThreshold() {
        return m_horizontal_threshold.getDouble(5.);
    }
}
