package frc.robot.Constants;

public class VisionConstants {

    public static double deadband = 0.3;
    public static double rotationalDeadband = 0.1;

    public static double radianMeasurement = 3.14159 / 180.0;

    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double limelightMountAngleDegrees = 16.6; 

    // distance from the center of the Limelight lens to the floor
    public static double limelightLensHeightInches = 19; 

    // distance from the target to the floor
    public static double goalHeightInches = 58; 

    //constants for shift command
    public static double Shift_P = 0.0244;
    public static double Shift_I = 0;
    public static double Shift_D = 0;

    //constants for HoldAngle command
    public static double HoldAngle_P = 0.027;
    public static double HoldAngle_I = 0;
    public static double HoldAngle_D = 0;

    //constants for AutoMove command
    public static double AutoMove_P = 0.3;
    public static double AutoMove_I = 0;
    public static double AutoMove_D = 0.003;

    public static double turningSpeed = 0.5;
    public static double horizontalSpeed = 1.5;
    public static double verticalSpeed = -1.5;

    //constants for ApproachApriltag command
    public static double ApproachApriltag_P = 0.9;
    public static double ApproachApriltag_I = 0;
    public static double ApproachApriltag_D = 0.001;

    public static double inchesToMeters = 39.3701;
    public static double inaccuracy = 0.1795;
    
}
