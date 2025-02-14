package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
 import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TunerConstants;


//This class holds all of the necessary functions and code for pathplanner to work and communicat with the robot
public class PathPlanner{

   public static CommandSwerveDrivetrain m_drivetrain;


   public static RobotConfig m_config;
   public static Pose2d m_position = new Pose2d(new Translation2d(), new Rotation2d(
    TunerConstants.defaultPathPlannerAngle)
    );

   public static SwerveRequest m_request = new SwerveRequest.ApplyRobotSpeeds();
   
    public PathPlanner(){

       AutoBuilder.configure(
         () -> m_drivetrain.getState().Pose,// Robot pose supplier
         (Pose2d) -> m_drivetrain.resetPose(m_position),// Method to reset odometry (will be called if your auto has a starting pose)
         ()-> {return m_drivetrain.getState().Speeds;},// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
         (speeds, feedforwards) -> m_drivetrain.setControl(m_request), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants (most likely will need tuning)
                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants (most likely would need tuning)
         ),
         m_config, // The robot configuration
         () -> {
//           // Boolean supplier that controls when the path will be mirrored for the red alliance
//           // This will flip the path being followed to the red side of the field.
//           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

           var alliance = DriverStation.getAlliance();
           if (alliance.isPresent()) {
             return alliance.get() == DriverStation.Alliance.Red;
           }
           return false;
         },
         m_drivetrain // Reference to this subsystem to set requirements
 );

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
