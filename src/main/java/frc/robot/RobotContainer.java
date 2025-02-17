// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.ApproachApriltag;
import frc.robot.Commands.AutoMove;
import frc.robot.Commands.HoldAngle;
import frc.robot.Commands.Shift;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShuffleboardHelper;
import frc.robot.subsystems.Vision;

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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision();

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

        ControllerConstants.driverController.a().whileTrue(new AutoMove(drivetrain, vision, CommandSwerveDrivetrain.AutoMoveAction.MOVE_VERTICAL));
        ControllerConstants.driverController.x().whileTrue(new HoldAngle(drivetrain, vision, ControllerConstants.driverController, MaxSpeed));
        //[]\ControllerConstants.driverController.b().whileTrue(new AutoMove(drivetrain, vision, CommandSwerveDrivetrain.AutoMoveAction.MOVE_HORIZONTAL));
        ControllerConstants.driverController.b().whileTrue(new Shift(drivetrain, vision, ControllerConstants.driverController, MaxSpeed));
        // ControllerConstants.driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-ControllerConstants.driverController.getLeftY(), -ControllerConstants.driverController.getLeftX()))
        // ));
        ControllerConstants.driverController.y().whileTrue(new ApproachApriltag(drivetrain, vision, 13, 3.0));

        // Run SysId routines when holding back/start and X/Y.

        // Note that each routine should be run exactly once in a single log.
        ControllerConstants.driverController.back().and(ControllerConstants.driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        ControllerConstants.driverController.back().and(ControllerConstants.driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        ControllerConstants.driverController.start().and(ControllerConstants.driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        ControllerConstants.driverController.start().and(ControllerConstants.driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // reset the field-centric heading on left bumper press
        ControllerConstants.driverController.leftBumper().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
