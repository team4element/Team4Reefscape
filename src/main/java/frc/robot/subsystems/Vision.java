// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
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
    TWO_DIMENSIONAL,
    THREE_DIMENSIONAL,
  }

  public Vision() {
    switchPipeline(Pipeline.TWO_DIMENSIONAL);

  }

  @Override
  public void periodic() {
    if (hasTarget()) {
      double angleToGoalDegrees = VisionConstants.limelightMountAngleDegrees + getVerticalOffset();
      double angleToGoalRadians = angleToGoalDegrees * VisionConstants.radianMeasurement;

      // calculate distance
      lastKnownTargetDistanceInches = (VisionConstants.goalHeightInches - VisionConstants.limelightLensHeightInches)
          / Math.tan(angleToGoalRadians);
    }
    // System.out.println(distanceFromLimelightToGoalInches);
  }

  // public double VerticalOffset(){

  // double offset;

  // if(hasTarget()){
  // offset = getVerticalOffset();
  // }
  // // else {
  // // offset = 0;
  // // }
  // return offset;
  // }

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

}
