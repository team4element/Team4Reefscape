package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.Pigeon2;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
 import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.Constants.TunerConstants;

public class PathPlanner{

   public static CommandSwerveDrivetrain drivetrain;

  //  public Pigeon2 pigeon = new Pigeon2(TunerConstants.kPigeonId);

   public static RobotConfig config;

  //  public SwerveDriveOdometry odometry = new SwerveDriveOdometry(null, null, null);
   
    public PathPlanner(){

//       AutoBuilder.configure(
//         drivetrain.getState().Pose, // Robot pose supplier
//         drivetrain.getState(), // Method to reset odometry (will be called if your auto has a starting pose)
//         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
//         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
//                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
//         ),
//         config, // The robot configuration
//         () -> {
//           // Boolean supplier that controls when the path will be mirrored for the red alliance
//           // This will flip the path being followed to the red side of the field.
//           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//           var alliance = DriverStation.getAlliance();
//           if (alliance.isPresent()) {
//             return alliance.get() == DriverStation.Alliance.Red;
//           }
//           return false;
//         },
//         this // Reference to this subsystem to set requirements
// );

  //        AutoBuilder.configure( 
  //           () -> drivetrain.pose2d, // Robot pose supplier
  //           odometry.resetPosition(pigeon.getRotation2d(),), // Method to reset odometry (will be called if your auto has a starting pose)
  //           drivetrain.getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //           (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
  //           new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
  //                   new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
  //                   new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
  //           ),
  //           config, // The robot configuration
  //           () -> {
  //             // Boolean supplier that controls when the path will be mirrored for the red alliance
  //             // This will flip the path being followed to the red side of the field.
  //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  //             var alliance = DriverStation.getAlliance();
  //             if (alliance.isPresent()) {
  //               return alliance.get() == DriverStation.Alliance.Red;
  //             }
  //             return false;
  //           },
  //           this // Reference to this subsystem to set requirements
  //   );
     }

    }
