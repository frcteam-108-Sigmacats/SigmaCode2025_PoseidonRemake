package frc.robot.subsystems.Climber;

public class ClimberConstants {
  // The IDS of all components on the climber mechanism
  public static int climberPivotID = 9;
  public static int climberIntakeID = 10;
  public static int servoMotorID = 0;
  public static int cageReaderID = 0;

  // The current limit to the rev motors
  public static int climberPivotCurrentLimit = 40;
  public static int climberIntakeCurrentLimit = 40;

  // The PID values for the Position Controller to position the climber
  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;

  // Position for the Servo to Unhinge
  public static double unHingeServoPos = 0.5;
  // Position for the Servo to Hinge
  public static double hingeServoPos = 0;

  // Position for climber to actuate to cage intake position
  public static double climberOutPosition = 62;
  // Position for climber to bring to prime climb position
  public static double climberInPosition = 5;
  // Position for climber in rest position
  public static double climberStowPosition = 0.0;
  // Speed for moving the climber out the robot frame perimeter
  public static double climberOutSpeed = 0.5;
  // Speed cor moving the climber in the robot frame perimeter
  public static double climberInSpeed = -0.5;
  // Speed for intaking the cage bar into the climber
  public static double cageIntakeSpeed = 0.8;
  // Speed for spitting out the cage bar from the climber
  public static double cageOuttakeSpeed = -0.5;
}
