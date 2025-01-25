// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

  public enum LedState {
    ON,
    OFF,
    BLINK
  }

  public Vision() {
    switchPipeline(0);
  }

  @Override
  public void periodic() {
    // Basic targeting data
    double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    System.out.println(String.format("tx: %f ty: %f ta: %f hasTarget: %b", tx, ty, ta, hasTarget));
    //System.out.println("hello limelight!");

    if (hasTarget){

      System.out.println("I can see");
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
   * @param pipeline The pipeline you want to switch to
   */
  public void switchPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex("", 0);
  }

}
