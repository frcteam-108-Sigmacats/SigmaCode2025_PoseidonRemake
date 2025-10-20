package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public double leftLLTX = 0.0;
    public double leftLLTY = 0.0;
    public double rightLLTX = 0.0;
    public double rightLLTY = 0.0;
    public Pose2d leftLLBotPoseEstimate = new Pose2d();
    public double leftLLBotPoseTimeStamp = 0.0;
    public Pose2d rightLLBotPoseEstimate = new Pose2d();
    public double rightLLBotPoseTimeStamp = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setRobotOrientation(double degrees, double angleSpeed) {}
}
