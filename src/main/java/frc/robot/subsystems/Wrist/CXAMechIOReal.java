package frc.robot.subsystems.Wrist;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class CXAMechIOReal implements CXAMechIO {
  // Instantiates the motors in the mechanism
  private TalonFX cxaMotor;
  private SparkBase wristMotor;
  private SparkBase hopperMotor;

  // Creates the config objects to implement into the motors
  private TalonFXConfiguration cxaMotorConfig = new TalonFXConfiguration();
  private VelocityVoltage velocity = new VelocityVoltage(0);
  private SparkBaseConfig wristMotorConfig = new SparkFlexConfig();
  private SparkBaseConfig hopperMotorConfig = new SparkMaxConfig();

  // Instantiate a PID Controller to control Wrist Pivot
  private SparkClosedLoopController wristPivotPID;

  // Instantiates the Absolute Encoder for tracking Wrist Pivot
  private AbsoluteEncoder wristPivotEnc;

  // Instantiates the CANRange used to detect the coral
  private CANrange canRange;

  // Create config object to apply to CANRange
  private CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();

  // Creates Alerts to notify if any component is disconnected
  private Alert cxaMotorDisconnected;
  private Alert hopperMotorDisconnected;
  private Alert wristMotorDisconnected;
  private Alert wristEncoderDisconnected;
  private Alert canRangeDisconnected;

  public CXAMechIOReal() {
    // Assigning the IDs to all hardware components in the mechanism
    cxaMotor = new TalonFX(CXAMechConstants.cXAMotorID, "*");
    hopperMotor = new SparkMax(CXAMechConstants.hopperMotorID, MotorType.kBrushless);
    wristMotor = new SparkFlex(CXAMechConstants.wristMotorID, MotorType.kBrushless);

    canRange = new CANrange(CXAMechConstants.canRangeID, "*");

    // Configuring the Motors

    // CoralXAlgae Motor Configs

    // Neutral Mode for Motor
    cxaMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Current Limit Configs
    cxaMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    cxaMotorConfig.CurrentLimits.StatorCurrentLimit = CXAMechConstants.cxaMotorCurrentLimit;
    // Normal Velocity Configs for coral manipulation
    cxaMotorConfig.Slot0.kP = CXAMechConstants.velocityP;
    cxaMotorConfig.Slot0.kI = CXAMechConstants.velocityI;
    cxaMotorConfig.Slot0.kD = CXAMechConstants.velocityD;
    // Fast Velocity Configs for shooting Algae
    cxaMotorConfig.Slot1.kP = CXAMechConstants.fastVelocityP;
    cxaMotorConfig.Slot1.kI = CXAMechConstants.fastVelocityI;
    cxaMotorConfig.Slot1.kD = CXAMechConstants.fastVelocityD;

    // Hopper Motor Config
    // Current Limit and Idle Mode Configs
    hopperMotorConfig.smartCurrentLimit(CXAMechConstants.coralHopperMotorCurrentLimit);
    hopperMotorConfig.idleMode(IdleMode.kCoast);

    // Wrist Motor Configs
    // Current Limit and Idle Mode Configs
    wristMotorConfig.smartCurrentLimit(CXAMechConstants.coralAlgaeWristCurrentLimit);
    wristMotorConfig.idleMode(IdleMode.kBrake);

    // PID Configs
    wristMotorConfig.closedLoop.pidf(
        CXAMechConstants.pivotP,
        CXAMechConstants.pivotI,
        CXAMechConstants.pivotD,
        CXAMechConstants.pivotFF);
    wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristMotorConfig.closedLoop.positionWrappingEnabled(true);
    wristMotorConfig.closedLoop.positionWrappingInputRange(0, 360);

    // Configuring CANRange
    canRangeConfigs.FovParams.FOVRangeY = 11;
    canRangeConfigs.FovParams.FOVRangeX = 11;
    canRangeConfigs.FovParams.FOVCenterY = 10;
    canRangeConfigs.ProximityParams.ProximityThreshold = 0.12;
    // Assigning the Absolute Encoder to the Wrist Motor Plugged in Absolute Encoder
    wristPivotEnc = wristMotor.getAbsoluteEncoder();

    // Assinging the PID Controller to the built in PID in the Wrist Motor
    wristPivotPID = wristMotor.getClosedLoopController();

    // Adding Configs to the motors
    cxaMotor.getConfigurator().apply(new TalonFXConfiguration());
    cxaMotor.getConfigurator().apply(cxaMotorConfig);
    tryUntilOk(
        hopperMotor,
        5,
        () ->
            hopperMotor.configure(
                hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        wristMotor,
        5,
        () ->
            wristMotor.configure(
                wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Applying Configs to CANRange
    canRange.getConfigurator().apply(new CANrangeConfiguration());
    canRange.getConfigurator().apply(canRangeConfigs);

    // Sets up the Alert System
    cxaMotorDisconnected = new Alert("CXA Motor Disconnected", AlertType.kError);
    hopperMotorDisconnected = new Alert("Hopper Motor Disconnected", AlertType.kError);
    wristMotorDisconnected = new Alert("Wrist Motor Disconnected", AlertType.kError);
    wristEncoderDisconnected = new Alert("Wrist Abs Encoder Disconnected", AlertType.kError);
    canRangeDisconnected = new Alert("CAN Range Disconnected", AlertType.kError);
  }

  // Updates the inputs that will be sent to AdvantageScope
  @Override
  public void updateInputs(CXAMechIOInputs inputs) {
    // Assinging the inputs to be logged to CXA Motor readings
    inputs.cxaMotorCurrentAmps = cxaMotor.getStatorCurrent().getValueAsDouble();
    inputs.cxaMotorVoltage = cxaMotor.getMotorVoltage().getValueAsDouble();

    // Assigning the inputs to be logged to the Hopper Motor Readings
    inputs.hopperMotorCurrentAmps = hopperMotor.getOutputCurrent();
    inputs.hopperMotorVoltage = hopperMotor.getBusVoltage();

    // Assigning the inputs to be logged to the Wrist Motor Readings
    inputs.cxaWristCurrentAmps = wristMotor.getOutputCurrent();
    inputs.cxaWristVoltage = wristMotor.getBusVoltage();

    // Assinging the inputs to be logged to the absolute encoder readings and CAM Range Detection
    inputs.wristAngle = wristPivotEnc.getPosition();
    inputs.detectCoral = canRange.getIsDetected(true).getValue();

    // Updates on whether alert should pop up or not
    cxaMotorDisconnected.set(!cxaMotor.isConnected());
    hopperMotorDisconnected.set(hopperMotor.getFaults().can);
    wristMotorDisconnected.set(wristMotor.getFaults().can);
    canRangeDisconnected.set(canRange.isConnected());
    wristEncoderDisconnected.set(wristMotor.getFaults().sensor);
  }

  // Sets the Wrist to a position using PID
  @Override
  public void setWristPosition(double position) {
    wristPivotPID.setReference(position, ControlType.kPosition);
  }

  // Sets CXA Motor with a velocity using Velocity PID Controller
  @Override
  public void setCXAMotorVel(double vel, int slot) {
    cxaMotor.setControl(velocity.withVelocity(vel).withSlot(slot));
  }

  // Sets CXA Motor with Manual Speed
  @Override
  public void setCXAMotorSpeed(double speed) {
    cxaMotor.set(speed);
  }

  // Sets the Wrist Motor with Manual Speed
  @Override
  public void setWristSpeed(double speed) {
    wristMotor.set(speed);
  }

  // Checks to see if the CANRange Detects a coral
  @Override
  public boolean isThereCoral() {
    return canRange.getIsDetected(true).getValue();
  }
}
