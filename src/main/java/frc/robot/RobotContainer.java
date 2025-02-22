// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AutoMove;
import frc.robot.Commands.LevelSetPoints;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JawConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.ShuffleboardHelper;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Jaw.JawAction;
import frc.robot.subsystems.LowerJaw.LowerJawAction;
import frc.robot.subsystems.LowerJaw;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Subsystems?
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision m_vision = new Vision();
    public final Jaw m_jaw = new Jaw();
    public final LowerJaw m_lowerJaw = new LowerJaw();
    public final Elevator m_elevator = new Elevator();
  

    public RobotContainer() {
        configureBindings();
        ShuffleboardHelper.getInstance().initialize();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-ControllerConstants.driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-ControllerConstants.driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-ControllerConstants.driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        //Driver Controls
        ControllerConstants.driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        ControllerConstants.driverController.x().onTrue(new AutoMove(drivetrain, m_vision, CommandSwerveDrivetrain.AutoMoveAction.TURN_IN_PLACE));
        ControllerConstants.driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-ControllerConstants.driverController.getLeftY(), -ControllerConstants.driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        ControllerConstants.driverController.back().and(ControllerConstants.driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        ControllerConstants.driverController.back().and(ControllerConstants.driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        ControllerConstants.driverController.start().and(ControllerConstants.driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        ControllerConstants.driverController.start().and(ControllerConstants.driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // reset the field-centric heading on left bumper press
        ControllerConstants.driverController.leftBumper().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        //Operator COntrols
        ControllerConstants.operatorController.rightBumper().whileTrue(m_lowerJaw.c_intakeCoral(LowerJawAction.INTAKE_CORAL, JawConstants.intakeSpeed));
        ControllerConstants.operatorController.leftBumper().whileTrue(m_lowerJaw.c_intakeCoral(LowerJawAction.OUTTAKE_CORAL, JawConstants.intakeSpeed));

        ControllerConstants.operatorController.rightTrigger().whileTrue(m_jaw.c_intakeAlgae(JawAction.INTAKE_ALGAE, JawConstants.intakeSpeed));
        ControllerConstants.operatorController.leftTrigger().whileTrue(m_jaw.c_intakeAlgae(JawAction.OUTTAKE_ALGAE, JawConstants.intakeSpeed));

        ControllerConstants.operatorController.povUp().whileTrue( m_elevator.c_moveElevator(m_elevator, ElevatorConstants.manualSpeed));
        ControllerConstants.operatorController.povDown().whileTrue( m_elevator.c_moveElevator(m_elevator, ElevatorConstants.manualSpeed * -1));

        //ControllerConstants.operatorController.a().onTrue(new LevelSetPoints(m_elevator, ElevatorConstants.levelOneSetPoint));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }


    // private SequentialCommandGroup LevelOne(double rpmTop, double rpmBot, double timeout, double elevatorSpeed, double armAngle) {
    // return new SequentialCommandGroup(new LevelSetPoints(m_elevator, ElevatorConstants.levelOneSetPoint),
    
    // new SequentialCommandGroup(m_lowerJaw.c_intakeCoral(JawAction.OUTTAKE_CORAL, .5).withTimeout(1)));
    
    // }

}
