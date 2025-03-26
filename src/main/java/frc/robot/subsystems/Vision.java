// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

  public double lastKnownTargetDistanceInches;
  private CommandSwerveDrivetrain drivetrain=null;
  PoseEstimate measurement;

  public enum LedState {
    ON,
    OFF,
    BLINK
  }

  public enum Pipeline {
    CENTER,
    RIGHT_PIPE,
    LEFT_PIPE,
    THREE_DIMENSIONAL
  }

  public Vision(CommandSwerveDrivetrain drivetrain) {
    switchPipeline(Pipeline.THREE_DIMENSIONAL);
    this.drivetrain=drivetrain;
  }

  public void doPeriodically(){
    if (hasTarget()) {
      double angleToGoalDegrees = VisionConstants.limelightMountAngleDegrees + getVerticalOffset();
      double angleToGoalRadians = angleToGoalDegrees * VisionConstants.radianMeasurement;
      
      double heading=drivetrain.getState().Pose.getRotation().getDegrees();
      LimelightHelpers.SetRobotOrientation("", heading, 0, 0, 0, 0, 0);
      measurement=LimelightHelpers.getBotPoseEstimate_wpiBlue("");

      if(measurement!=null){
        if(measurement.pose.getX()!=0&&measurement.pose.getY()!=0&&measurement.avgTagDist<2.5){
          drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1*Math.pow(measurement.avgTagDist,2),0.1*Math.pow(measurement.avgTagDist,2),0.1*Math.pow(measurement.avgTagDist,2)));
          drivetrain.addVisionMeasurement(measurement.pose, Utils.fpgaToCurrentTime(measurement.timestampSeconds));
        }
      }
      // calculate distance
      lastKnownTargetDistanceInches = (VisionConstants.goalHeightInches - VisionConstants.limelightLensHeightInches)
          / Math.tan(angleToGoalRadians);

      System.out.println(currentPipeline());
    }
  }

  @Override
  public void periodic() {
    doPeriodically();
  }


  /**
   * Controls the LED on the limelight
   * 
   * @param led_state On, Off, BLink
   */
  public void controlLED(LedState led_state) {
    switch (led_state) {
      case ON:
        LimelightHelpers.setLEDMode_ForceOn("");
        break;
      case OFF:
        LimelightHelpers.setLEDMode_ForceOff("");
        break;
      case BLINK:
        LimelightHelpers.setLEDMode_ForceBlink("");
        break;
      default:
        LimelightHelpers.setLEDMode_ForceOff("");
        break;
    }
  }

  public PoseEstimate getMeasurement(){
    return measurement;
  }

  /**
   * Switch between pipelines
   * 
   * @param pipeline The pipeline you want to switch to
   */
  public void switchPipeline(Pipeline pipeline) {
    LimelightHelpers.setPipelineIndex("", pipeline.ordinal());
  }

  public void switchPipeline(int dir) {
    int pipeline_index = currentPipeline().ordinal() + dir;
    if(pipeline_index > Pipeline.RIGHT_PIPE.ordinal()){
      switchPipeline(Pipeline.CENTER);
    }else if(pipeline_index < Pipeline.CENTER.ordinal()){
      switchPipeline(Pipeline.RIGHT_PIPE);
    }else{
      switchPipeline(Pipeline.values()[pipeline_index]);
    }
  }

  public Pipeline currentPipeline(){
    int index = (int)LimelightHelpers.getCurrentPipelineIndex("");
    return Pipeline.values()[index];
  }

  // public Pose3d toPose3d(){
  //   Pose3d pose = new LimelightHelpers.toPose3D("");
  //   return pose;
  // }

  public double getHorizontalOffset() {
    return LimelightHelpers.getTX("");
  }

  public double getVerticalOffset() {
    // LimelightHelpers.getBotPose3d("");
    return -LimelightHelpers.getTY("");
  }

  public Pose3d getTarget3DPose() {
    return LimelightHelpers.getTargetPose3d_RobotSpace("");
  }

  public double[] getTarget2DPose() {
    return LimelightHelpers.getTargetPose_RobotSpace("");
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV("");
  }

  public Command c_ChangePipeline(Pipeline pipeline){
    return runOnce(() -> switchPipeline(pipeline));
  }

  public Command c_ChangePipeline(int dir){
    return runOnce(() -> switchPipeline(dir));
  }
}
