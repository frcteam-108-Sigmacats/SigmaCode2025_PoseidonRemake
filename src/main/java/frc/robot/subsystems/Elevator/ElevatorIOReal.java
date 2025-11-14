package frc.robot.subsystems.Elevator;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOReal implements ElevatorIO {
  // Instantiates the Elevator Motor Objects
  private SparkMax leftMotor;
  private SparkMax rightMotor;

  // Instantiate the Motor Configs
  private SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

  // Instantiates the encoder tracking Elevator's Position
  private RelativeEncoder elevatorEnc;

  // Instantiates PID Controller for Elevator Position Control
  private SparkClosedLoopController elevatorPID;

  public ElevatorIOReal() {
    // Assigns the Motor to correct IDs
    leftMotor = new SparkMax(ElevatorConstants.leftElevatorMotorID, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.rightElevatorMotorID, MotorType.kBrushless);

    // Configure the Right Motor

    // Setting the Right Elevator Idle Mode and Current Limit
    rightMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.smartCurrentLimit(ElevatorConstants.ElevatorMotorCurrentlimit);

    // Testing out internal encoder position
    rightMotorConfig.encoder.positionConversionFactor(1);
    // Setting the CPR of our Alternate Encoder for accurate readings
    // rightMotorConfig.alternateEncoder.countsPerRevolution(8192);
    // rightMotorConfig.alternateEncoder.inverted(
    //     true); // Inverting our encoder to read the position of the elevator the right way

    // Assinging the PID Values to the Right Elevator PID Controller
    rightMotorConfig.closedLoop.pid(
        ElevatorConstants.piviotP, ElevatorConstants.piviotI, ElevatorConstants.piviotD);
    rightMotorConfig.closedLoop.velocityFF(1 / 5676); // Setting the Feedforward Voltage
    rightMotorConfig.closedLoop.feedbackSensor(
        FeedbackSensor
            .kPrimaryEncoder); // Telling our PID Controller to use the Through Bore/ Internal
    // Encoder
    // Encoder connected to the motor as feedback

    // Setting the Elevator Forward and Reverse Limits to make sure our Robot does not go beyond its
    // limitations angle wise to not break the mechanism
    rightMotorConfig.softLimit.forwardSoftLimit(ElevatorConstants.elevatorForwardSoftLimit);
    rightMotorConfig.softLimit.reverseSoftLimit(ElevatorConstants.elevatorReverseSoftLimit);

    // Setting the Left Elevator Idle Mode and Current Limit
    leftMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.smartCurrentLimit(ElevatorConstants.ElevatorMotorCurrentlimit);

    // Setting the Left Elevator Motor Forward and Reverse Limits
    leftMotorConfig.softLimit.forwardSoftLimit(ElevatorConstants.elevatorForwardSoftLimit);
    leftMotorConfig.softLimit.reverseSoftLimit(ElevatorConstants.elevatorReverseSoftLimit);

    // Telling the Left Elevator Motor to follow the Right Elevator Motor
    leftMotorConfig.follow(rightMotor, true);

    // Setting the PID Controller variable to the right elevator motor internal PID Controller
    elevatorPID = rightMotor.getClosedLoopController();

    // Setting the Relative Encoder variable to the Right Elevator Motor External connection to the
    // Through Bore Encoder
    // elevatorEnc = rightMotor.getAlternateEncoder();
    elevatorEnc = rightMotor.getEncoder(); // Using the internal encoder

    // Add Configurations to Motors
    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    elevatorEnc.setPosition(0);
  }

  // Assigns inputs to be logged to component readings
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.leftElevatorMotorDisconnected = leftMotor.getFaults().can;
    inputs.rightElevatorMotorDiconnected = rightMotor.getFaults().can;
    // Sets Current Amps and Voltage to the motors readings
    inputs.leftElevatorMotorCurrentAmps = leftMotor.getOutputCurrent();
    inputs.leftElevatorMotorVoltage = leftMotor.getBusVoltage();
    inputs.rightElevatorMotorCurrentAmps = rightMotor.getOutputCurrent();
    inputs.rightElevatorMotorVoltage = rightMotor.getBusVoltage();

    // Assigned Elevator Position to the encoder readings
    inputs.elevatorPos = elevatorEnc.getPosition();
  }

  @Override
  public void setElevatorPosition(double position) {
    elevatorPID.setReference(position, ControlType.kPosition);
  }

  @Override
  public void setElevatorSpeed(double speed) {
    rightMotor.set(speed);
  }
}
