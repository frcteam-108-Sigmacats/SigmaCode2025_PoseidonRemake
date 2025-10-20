// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;

public class VisionMech extends SubsystemBase {
  private VisionIO io;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  /** Creates a new VisionMech. */
  public VisionMech(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.currentMode == Mode.REAL) {
      io.updateInputs(inputs);
      io.setRobotOrientation(Drive.visionPose().getRotation().getDegrees(), 0.01);
    }
  }

  public double getLeftLLTX() {
    return inputs.leftLLTX;
  }

  public double getLeftLLTY() {
    return inputs.leftLLTY;
  }

  public double getRightLLTX() {
    return inputs.rightLLTX;
  }

  public double getRightLLTY() {
    return inputs.rightLLTY;
  }

  public Pose2d getLeftLLBotPose() {
    return inputs.leftLLBotPoseEstimate;
  }

  public Pose2d getRightLLBotPose() {
    return inputs.rightLLBotPoseEstimate;
  }

  public double getLeftLLBotPoseTimeStamp() {
    return inputs.leftLLBotPoseTimeStamp;
  }

  public double getRightLLBotPoseTimeStamp() {
    return inputs.rightLLBotPoseTimeStamp;
  }
}
