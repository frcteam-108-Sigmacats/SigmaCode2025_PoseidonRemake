package frc.robot.subsystems.Vision;

import frc.robot.LimelightHelpers;

public class VisionIOReal implements VisionIO {
  private String leftLLName = "limelight-leftll";
  private String rightLLName = "limelight-rightll";

  public VisionIOReal() {}

  @Override
  public void setRobotOrientation(double angle, double angleSpeed) {
    // LimelightHelpers.SetRobotOrientation(leftLLName, angle, angleSpeed, 0, 0, 0, 0);
    // LimelightHelpers.SetRobotOrientation(rightLLName, angle, angleSpeed, 0, 0, 0, 0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.leftLLTX = LimelightHelpers.getTX(leftLLName);
    inputs.rightLLTX = LimelightHelpers.getTX(rightLLName);
    inputs.leftLLTY = LimelightHelpers.getTY(leftLLName);
    inputs.rightLLTY = LimelightHelpers.getTY(rightLLName);
    // inputs.leftLLBotPoseEstimate =
    //     LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(leftLLName).pose;
    // inputs.leftLLBotPoseTimeStamp =
    //     LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(leftLLName).timestampSeconds;
    // inputs.rightLLBotPoseEstimate =
    //     LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rightLLName).pose;
    // inputs.rightLLBotPoseTimeStamp =
    //     LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rightLLName).timestampSeconds;
  }
}
