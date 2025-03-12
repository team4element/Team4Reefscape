// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ApproachApriltag;
import frc.robot.Commands.ApriltagAllignment;
import frc.robot.Commands.AutoMove;
import frc.robot.Commands.BargeShot;
import frc.robot.Commands.ElevateAndPivot;
import frc.robot.Commands.HoldAngle;
import frc.robot.Commands.IntakeAlgae;
import frc.robot.Commands.Shift;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JawConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.UpperJaw;
import frc.robot.subsystems.ShuffleboardHelper;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain.AutoMoveAction;
import frc.robot.subsystems.Vision.Pipeline;
import frc.robot.subsystems.LowerJaw;
import frc.robot.subsystems.Pivot;

public class RobotContainer {

    SendableChooser<Command> sendableAuton;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.70).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
//.75

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(.4).withRotationalDeadband(.4) // Add a 10% deadband //0.3
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision m_vision     = new Vision();
    public final UpperJaw m_upperJaw = new UpperJaw();
    public final LowerJaw m_lowerJaw = new LowerJaw();
    public final Elevator m_elevator = new Elevator();
    public final Pivot    m_pivot    = new Pivot();
    public final Climb    m_climb    = new Climb();

    public RobotContainer() {

        NamedCommands.registerCommand("align to left side", new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.LEFT_PIPE));
        NamedCommands.registerCommand("align to right side", new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.RIGHT_PIPE));
        NamedCommands.registerCommand("level 1", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_1, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 2", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_2, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 3", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_3, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 4", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_4, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("outtake coral", m_lowerJaw.c_intakeCoral(-0.15).withTimeout(1));
        NamedCommands.registerCommand("Lower elevator", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_1, 1, 0).withTimeout(1.5));
        NamedCommands.registerCommand("Intake Algae", new IntakeAlgae(m_upperJaw, m_lowerJaw, 0.7, 0.7));
        // creates a menu on shuffle board for autons
        sendableAuton = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", sendableAuton);

        configureBindings();
        ShuffleboardHelper.getInstance().initialize();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(ControllerConstants.yTranslationModifier.apply(-ControllerConstants.driverController.getLeftY() * MaxSpeed * drivetrain.speedToDouble(drivetrain.m_speed))) // Drive forward with negative Y (forward)
                    .withVelocityY(ControllerConstants.xTranslationModifier.apply(-ControllerConstants.driverController.getLeftX() * MaxSpeed * drivetrain.speedToDouble(drivetrain.m_speed))) // Drive left with negative X (left)
                    .withRotationalRate(ControllerConstants.zRotationModifier.apply(-ControllerConstants.driverController.getRightX() * MaxAngularRate * drivetrain.speedToDouble(drivetrain.m_speed))) // Drive counterclockwise with negative X (left)
            )
        );

        m_pivot.setDefaultCommand(m_pivot.c_pivotManual());
        m_elevator.setDefaultCommand(m_elevator.c_moveElevator());

	    //Driver Controls
        ControllerConstants.driverController.x().whileTrue(new HoldAngle(drivetrain, m_vision, ControllerConstants.driverController, MaxSpeed, MaxAngularRate));
        ControllerConstants.driverController.y().whileTrue(new ApproachApriltag(drivetrain, m_vision, 0.5));
        ControllerConstants.driverController.a().whileTrue(new ApriltagAllignment(drivetrain, m_vision, 0.5, ControllerConstants.driverController, MaxSpeed, MaxSpeed));
        ControllerConstants.driverController.b().whileTrue(new AutoMove(drivetrain, m_vision, AutoMoveAction.MOVE_VERTICAL));

        ControllerConstants.driverController.leftBumper().onTrue(drivetrain.c_seedFieldRelative());
      //  ControllerConstants.driverController.leftTrigger().whileTrue(new ClimbDown(m_climb, 1));
      // ControllerConstants.driverController.rightTrigger().whileTrue(new ClimbUp(m_climb, 1));

        ControllerConstants.driverController.start().onTrue(drivetrain.c_updateSpeed(1));
        ControllerConstants.driverController.back().onTrue(drivetrain.c_updateSpeed(-1));

        ControllerConstants.driverController.povLeft().whileTrue(new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.LEFT_PIPE));
        ControllerConstants.driverController.povRight().whileTrue(new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.RIGHT_PIPE));
        ControllerConstants.driverController.povUp().onTrue(new Vision().c_ChangePipeline(1));
        ControllerConstants.driverController.povDown().onTrue(new Vision().c_ChangePipeline(-1));

        //Operator Controls
        ControllerConstants.operatorController.leftBumper().whileTrue(m_lowerJaw.c_intakeCoral(JawConstants.intakeSpeed));
        ControllerConstants.operatorController.rightBumper().whileTrue(m_lowerJaw.c_intakeCoral(JawConstants.topOuttakeSpeed));
        ControllerConstants.operatorController.leftTrigger().whileTrue(new IntakeAlgae(m_upperJaw, m_lowerJaw, .7, .7));
        ControllerConstants.operatorController.rightTrigger().whileTrue(new IntakeAlgae(m_upperJaw, m_lowerJaw, JawConstants.topOuttakeSpeed, JawConstants.bottomOuttakeSpeed));

        ControllerConstants.operatorController.a().whileTrue(new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_1, 0, 0));
        ControllerConstants.operatorController.b().whileTrue(new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_2, 0 ,0));
        ControllerConstants.operatorController.y().whileTrue(new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_3, 0 ,0));
        ControllerConstants.operatorController.x().whileTrue(new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.CORAL_STATION, 0, 0));
        
        ControllerConstants.operatorController.back().whileTrue(new BargeShot(m_pivot, m_upperJaw, m_lowerJaw));
      //  ControllerConstants.operatorController.back().whileTrue(new MovingToBarge(m_elevator, m_pivot, m_upperJaw, m_lowerJaw));
        ControllerConstants.operatorController.start().whileTrue(m_pivot.c_goToSetPoint(Elevator.Level.ALGAE, 1));

    }

    public Command getAutonomousCommand() {
        return sendableAuton.getSelected();
    }

    public Command c_fieldRelative(){
        return drivetrain.applyRequest(() -> drive);
    }

    public void onEnable(){
        m_pivot.resetPivotEncoder();
        m_elevator.resetEncoders();
        m_climb.reset();
    }
}
