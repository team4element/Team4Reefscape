// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.AngleToReef;
import frc.robot.Commands.BargeShot;
import frc.robot.Commands.ClimbDown;
import frc.robot.Commands.ClimbHold;
import frc.robot.Commands.ClimbUp;
import frc.robot.Commands.ElevateAndPivot;
import frc.robot.Commands.IntakeAlgae;
import frc.robot.Commands.Shift;
import frc.robot.Commands.alignToBarge;
import frc.robot.Commands.oldClimb;
import frc.robot.Commands.recordValues;
import frc.robot.Commands.reefHorizontal;
import frc.robot.Commands.turnToStation;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JawConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.UpperJaw;
import frc.robot.subsystems.ShuffleboardHelper;
import frc.robot.subsystems.Vision;
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
            .withDeadband(1).withRotationalDeadband(1) // Add a 10% deadband //0.3
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective); // Use open-loop control for drive motors


            private final SwerveRequest.RobotCentric rcdrive = new SwerveRequest.RobotCentric()
            .withDeadband(1).withRotationalDeadband(1) // Add a 10% deadband //0.3
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Telemetry logger = new Telemetry(MaxSpeed);
 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision m_vision     = new Vision(drivetrain);
    public final UpperJaw m_upperJaw = new UpperJaw();
    public final LowerJaw m_lowerJaw = new LowerJaw();
    public final Elevator m_elevator = new Elevator();
    public final Pivot    m_pivot    = new Pivot();
    public final Climb    m_climb    = new Climb();

    private double teleStart;
    private double autoStart;

    public RobotContainer() {

        NamedCommands.registerCommand("align to left side", new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.LEFT_PIPE));
        NamedCommands.registerCommand("align to right side", new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.RIGHT_PIPE));
        NamedCommands.registerCommand("level 1", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_1, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 2", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_2, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 3", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_3, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 4", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_4, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("CoralStation", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.CORAL_STATION, 0, 0).withTimeout(2));
        NamedCommands.registerCommand("outtake coral", m_lowerJaw.c_intakeCoral(-0.12).withTimeout(1.5));
        NamedCommands.registerCommand("Lower elevator", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_1, 1, 0).withTimeout(1.5));
        NamedCommands.registerCommand("Intake Algae", new IntakeAlgae(m_upperJaw, m_lowerJaw, 0.7, 0.7));
        NamedCommands.registerCommand("IntakeCoral", m_lowerJaw.c_intakeCoral(JawConstants.intakeSpeed).withTimeout(1.5));
        // creates a menu on shuffle board for autons
        sendableAuton = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", sendableAuton);
        Commands.run(()->logger.telemeterize(drivetrain.getState())).ignoringDisable(true).schedule();

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
         m_climb.setDefaultCommand(new ClimbHold(m_climb));
        m_elevator.setDefaultCommand(m_elevator.c_moveElevator());
	    //Driver Controls
        //ControllerConstants.driverController.x().whileTrue(new HoldAngle(drivetrain, m_vision, ControllerConstants.driverController, MaxSpeed, MaxAngularRate));
        // ControllerConstants.driverController.a().whileTrue(new AngleToReef(drivetrain, ControllerConstants.driverController, MaxSpeed));
        //  //ControllerConstants.driverController.y().whileTrue(new ApproachApriltag(drivetrain, m_vision, 0.5));
        //  ControllerConstants.driverController.y().whileTrue(m_climb.c_horizontal(73.59, 74.46));
        // // ControllerConstants.driverController.a().whileTrue(new ApriltagAllignment(drivetrain, m_vision, 0.5, ControllerConstants.driverController, MaxSpeed, MaxSpeed));
        //  ControllerConstants.driverController.b().whileTrue(new AutoMove(drivetrain, m_vision, AutoMoveAction.MOVE_VERTICAL_RIGHT));
        //  ControllerConstants.driverController.x().whileTrue(new AutoMove(drivetrain, m_vision, AutoMoveAction.MOVE_VERTICAL_LEFT));

        ControllerConstants.driverController.y().whileTrue(new turnToStation(drivetrain, MaxAngularRate));
        ControllerConstants.driverController.x().whileTrue(new reefHorizontal(m_vision, drivetrain, MaxSpeed, ControllerConstants.driverController));
        ControllerConstants.driverController.b().whileTrue(new reefHorizontal(m_vision, drivetrain, MaxSpeed, ControllerConstants.driverController));
        ControllerConstants.driverController.rightStick().whileTrue(new AngleToReef(drivetrain, ControllerConstants.driverController, MaxSpeed));

        ControllerConstants.driverController.rightBumper().onTrue(drivetrain.c_seedFieldRelative());
        ControllerConstants.driverController.leftBumper().whileTrue(
            drivetrain.applyRequest(()->rcdrive.withVelocityX(ControllerConstants.yTranslationModifier.apply(-ControllerConstants.driverController.getLeftY() * MaxSpeed * drivetrain.speedToDouble(drivetrain.m_speed))) // Drive forward with negative Y (forward)
            .withVelocityY(ControllerConstants.xTranslationModifier.apply(-ControllerConstants.driverController.getLeftX() * MaxSpeed * drivetrain.speedToDouble(drivetrain.m_speed))) // Drive left with negative X (left)
             .withRotationalRate(ControllerConstants.zRotationModifier.apply(-ControllerConstants.driverController.getRightX() * MaxAngularRate * drivetrain.speedToDouble(drivetrain.m_speed)))) // Drive counterclockwise with negative X (left))
         );
         ControllerConstants.driverController.leftTrigger().whileTrue(new ClimbDown(m_climb, .45));
         ControllerConstants.driverController.rightTrigger().whileTrue(new ClimbUp(m_climb, .45));

        ControllerConstants.driverController.start().onTrue(drivetrain.c_updateSpeed(1));
        ControllerConstants.driverController.back().onTrue(drivetrain.c_updateSpeed(-1));
        //ControllerConstants.driverController.button(10).onTrue();

        // ControllerConstants.driverController.povLeft().whileTrue();
         // ControllerConstants.driverController.povRight().whileTrue(new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.RIGHT_PIPE));
       ControllerConstants.driverController.povUp().onTrue(m_vision.c_ChangePipeline(1));
        ControllerConstants.driverController.povDown().onTrue(m_vision.c_ChangePipeline(-1));
        ControllerConstants.driverController.povLeft().whileTrue(new oldClimb(m_climb, .45));
        ControllerConstants.driverController.povRight().whileTrue(new alignToBarge(drivetrain, MaxSpeed));

        //Operator Controls
        ControllerConstants.operatorController.leftBumper().whileTrue(m_lowerJaw.c_intakeCoral(JawConstants.intakeSpeed));
        ControllerConstants.operatorController.rightBumper().whileTrue(m_lowerJaw.c_intakeCoral(JawConstants.topOuttakeSpeed));

         ControllerConstants.operatorController.povUp().whileTrue(m_lowerJaw.c_intakeCoral(JawConstants.topOuttakeSpeed/2));
         ControllerConstants.operatorController.povDown().whileTrue(new recordValues(m_elevator, m_pivot));

        ControllerConstants.operatorController.leftTrigger().whileTrue(new IntakeAlgae(m_upperJaw, m_lowerJaw, .7, .7));
       ControllerConstants.operatorController.rightTrigger().whileTrue(new IntakeAlgae(m_upperJaw, m_lowerJaw, JawConstants.topOuttakeSpeed, JawConstants.bottomOuttakeSpeed));

         ControllerConstants.operatorController.a().whileTrue(new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_1, 0, 0));
         ControllerConstants.operatorController.b().whileTrue(new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_2, 0 ,0));
         ControllerConstants.operatorController.y().whileTrue(new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_3, 0 ,0));
         ControllerConstants.operatorController.x().whileTrue(new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.CORAL_STATION, 0, 0));
        
          ControllerConstants.operatorController.back().whileTrue(new BargeShot(m_pivot, m_upperJaw, m_lowerJaw));
        // ControllerConstants.operatorController.back().whileTrue(new ToBargeHeight(m_elevator, m_pivot, 0, m_upperJaw, m_lowerJaw));
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