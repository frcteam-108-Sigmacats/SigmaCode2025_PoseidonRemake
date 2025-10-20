package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberAngle = 0.0;
    public double climberPivotMotorVoltage = 0.0;
    public double climberIntakeMotorVoltage = 0.0;
    public double servoPosition = 0.0;
    public boolean cageReader = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setClimberPosition(double angle) {}

  public default void setClimberPivotSpeed(double speed) {}

  public default double getClimberAngle() {
    return 0.0;
  }

  public default void setClimberIntakeSpeed(double speed) {}

  public default boolean isCageIn() {
    return false;
  }

  public default void setServoPosition(double position) {}
}
