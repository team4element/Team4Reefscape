package frc.robot.Constants;

//import com.ctre.phoenix6.hardware.TalonFX;

public class JawConstants {
    public static final int TopId = 13;
    public static final int innerBottomId = 14;
    public static final int outerBottomId = 15;
    public static final int jawPivotId = 16;

    public static final double limitForward = 1;
    public static final double limitBackwards = -1;

    public static final double statorLimit = 80;
    public static final double supplyLimit = 80;

    public static final double lowerStatorLimit = 80;
    public static final double lowerSupplyLimit = 80;

    public static final double intakeSpeed = .1;
    public static final double bottomOuttakeSpeed = -.1;
    public static final double topOuttakeSpeed = -.1;
    

    public static final double rampUpTime = .8; //Ramp up time for future shooting of the algae to the net

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final String tableP = "Jaw P";
    public static final String tableI = "Jaw I";
    public static final String tableD = "Jaw D";
}
