package frc.robot.subsystems.Climber;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class ClimberIOReal implements ClimberIO {
  // Instantiating the motor hooked on the climber
  private SparkMax climberPivotMotor;
  private SparkMax climberIntakeMotor;

  // Instantiating the Configs for each rev motor
  private SparkMaxConfig climberPivotConfig = new SparkMaxConfig();
  private SparkMaxConfig climberIntakeConfig = new SparkMaxConfig();

  // Instantiating the servo motor used to unhinge the climber's clamp
  private Servo climberServo;
  // Instantiating the Abs Encoder that reads the angle of the climber
  private AbsoluteEncoder absEncoder;
  // Instantiating the limit switch used to detect if robot has a hold of the cage
  private DigitalInput cageReader;

  // Instantiating the PIDController to control the position of the climber
  private SparkClosedLoopController climberPIDController;

  public ClimberIOReal() {
    // Assigning the IDS to the components on the climber
    climberPivotMotor = new SparkMax(ClimberConstants.climberPivotID, MotorType.kBrushless);
    climberIntakeMotor = new SparkMax(ClimberConstants.climberIntakeID, MotorType.kBrushless);
    climberServo = new Servo(ClimberConstants.servoMotorID);
    cageReader = new DigitalInput(ClimberConstants.cageReaderID);

    // Configuring the climberPivotMotor
    climberPivotConfig.idleMode(IdleMode.kBrake);
    climberPivotConfig.smartCurrentLimit(ClimberConstants.climberPivotCurrentLimit);

    // Configuring the Absolute Encoder connected to the motor
    climberPivotConfig.absoluteEncoder.positionConversionFactor(
        360); // Switching the angle from 0-1 rotation to 0-360 degrees
    // Configuring the PID Controller that will be used to position the climber
    climberPivotConfig.closedLoop.pid(
        ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD); // Setting the PID
    climberPivotConfig.closedLoop.feedbackSensor(
        FeedbackSensor
            .kAbsoluteEncoder); // Telling the Position controller which encoder it will be getting
    // input from
    climberPivotConfig.closedLoop.positionWrappingEnabled(
        true); // Enabling position wrapping where the controller will calculate which direction
    // will be quicker to reach the desired angle
    climberPivotConfig.closedLoop.positionWrappingInputRange(
        0, 360); // Limits the position wrapping between 0 and 360 degrees
    // Configuring Soft (Digital) limit for the motor
    climberPivotConfig.softLimit.reverseSoftLimit(
        0); // Will make sure the motor does not go in that certain direction if the position of the
    // encoder is 0

    // Assigning the Abs Enc and PID Controller to the built in one in the motor and encoder
    // connected to the motor data port
    absEncoder = climberPivotMotor.getAbsoluteEncoder();
    climberPIDController = climberPivotMotor.getClosedLoopController();

    // Configuring the Climber Intake motor
    climberIntakeConfig.idleMode(IdleMode.kCoast);
    climberIntakeConfig.smartCurrentLimit(ClimberConstants.climberIntakeCurrentLimit);
    climberIntakeConfig.inverted(
        true); // Changes the direction in which is positive and which is negative

    // Applying the motor configurations 5 times to make sure the configurations are applied
    tryUntilOk(
        climberIntakeMotor,
        5,
        () ->
            climberIntakeMotor.configure(
                climberIntakeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    tryUntilOk(
        climberPivotMotor,
        5,
        () ->
            climberPivotMotor.configure(
                climberPivotConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }
  // Updates all the inputs we want to log to show up in AdvantageScope
  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberPivotMotorDisconnected = climberPivotMotor.getFaults().can;
    inputs.climberIntakeMotorDisconnected = climberIntakeMotor.getFaults().can;
    // Applying the voltages to the specific motors outputs
    inputs.climberPivotMotorVoltage = climberPivotMotor.getAppliedOutput();
    inputs.climberIntakeMotorVoltage = climberIntakeMotor.getAppliedOutput();
    // Grabbing the angle of the climber from te absolute encoder attached to the pivot
    inputs.climberAngle = absEncoder.getPosition();
    // Grabbing the position of the servo from the servo
    inputs.servoPosition = climberServo.getPosition();
    // Grabbing if the limit switch is being activated or not (pressed or not pressed)
    inputs.cageReader = cageReader.get();
  }

  @Override
  public void setClimberPosition(double angle) {
    climberPIDController.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void setClimberPivotSpeed(double speed) {
    climberPivotMotor.set(speed);
  }

  @Override
  public double getClimberAngle() {
    return absEncoder.getPosition();
  }

  @Override
  public void setClimberIntakeSpeed(double speed) {
    climberIntakeMotor.set(speed);
  }

  @Override
  public void setServoPosition(double position) {
    climberServo.set(position);
  }

  @Override
  public boolean isCageIn() {
    return cageReader.get();
  }
}
