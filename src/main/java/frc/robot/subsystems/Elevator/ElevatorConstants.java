package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
  // Elevator Motor IDs
  public static final int leftElevatorMotorID = 11;
  public static final int rightElevatorMotorID = 12;

  // Elevator Current Limit
  public static final int ElevatorMotorCurrentlimit = 40;

  // Elevator PID Control
  public static final double piviotP = 1.0;
  public static final double piviotI = 0.0;
  public static final double piviotD = 0.0;

  // Elevator Soft Limits
  public static final double elevatorForwardSoftLimit = 5.3; /*change when robo done */
  public static final double elevatorReverseSoftLimit = 0;

  // Elevator Set Points
  public static final double L1Pose = 0.0;
  public static final double L2Pose = 1.55;
  public static final double L3Pose = 2.8; // 2.4 comp old was 2.5
  public static final double L4Pose = 4.7; // ORIGINAL 5.1
  public static final double NetPose = 5.1;
  public static final double A1Pose = 0.6; // Get actual position for it
  public static final double A2Pose = 1.8; // Get actual position for it
}
