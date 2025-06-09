package frc.robot.subsystems.Wrist;

public class CXAMechConstants {
  // Motor IDS
  public static final int cXAMotorID = 13;
  public static final int hopperMotorID = 14;
  public static final int wristMotorID = 15;

  // CANRange ID
  public static final int canRangeID = 1;

  // Wrist Pivot PID Values
  public static final double pivotP = 0.01;
  public static final double pivotI = 0.0;
  public static final double pivotD = 0.0;
  public static final double pivotFF = 1 / 5676;

  // CXA Motor Regular Velocity Values
  public static final double velocityP = 0.001;
  public static final double velocityI = 0.0;
  public static final double velocityD = 0.0;

  // CXA Motor Fast Velocity Values
  public static final double fastVelocityP = 0.01;
  public static final double fastVelocityI = 0.0;
  public static final double fastVelocityD = 0.0;

  // Motor Current Limits
  public static final int cxaMotorCurrentLimit = 40;
  public static final int coralHopperMotorCurrentLimit = 45;
  public static final int coralAlgaeWristCurrentLimit = 35;

  // Ejecting Coral Speed
  public static final double coralEjectVelocity = -2000;

  // Coral Feeding Speed
  public static final double cxaMotorFeedVelocity = -5000;
  // Algae Removal Speed
  public static final double cxaMotorAlgaeRemovalVelocity = -3000;

  // Algae Hold Speed
  public static final double cxaMotorAlgaeHoldVelocity = -3000;

  // Ejects Algae
  public static final double algaeEjectVelocity = 6000;
  // Hopper Speed
  public static final double coralHopperSpeed = 1.0;

  // Wrist SetPoints
  public static final double l4WristPosition = 0; // ORIGINAL 40
  public static final double algaeRemovalWristPosition = 25; // Get the actual position for it
  public static final double algaeScorePrimePosition = 340;
  public static final double algaeScoreEndPosition = 350;
  public static final double restWristPosition = 0;
  public static final double algaeRestPosition = 10;
  public static final double feederPosition = 358;
}
