// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

  public double lastKnownTargetDistanceInches;

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

  public Vision() {
    switchPipeline(Pipeline.CENTER);
  }

  @Override
  public void periodic() {
    if (hasTarget()) {
      double angleToGoalDegrees = VisionConstants.limelightMountAngleDegrees + getVerticalOffset();
      double angleToGoalRadians = angleToGoalDegrees * VisionConstants.radianMeasurement;

      // calculate distance
      lastKnownTargetDistanceInches = (VisionConstants.goalHeightInches - VisionConstants.limelightLensHeightInches)
          / Math.tan(angleToGoalRadians);

      System.out.println(currentPipeline());
    }
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
