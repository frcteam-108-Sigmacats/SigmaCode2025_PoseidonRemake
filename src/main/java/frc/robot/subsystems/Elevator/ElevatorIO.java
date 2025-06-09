package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double leftElevatorMotorVoltage = 0.0;
    public double leftElevatorMotorCurrentAmps = 0.0;
    public double rightElevatorMotorVoltage = 0.0;
    public double rightElevatorMotorCurrentAmps = 0.0;
    public double elevatorPos = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setElevatorPosition(double position) {}

  public default void setElevatorSpeed(double speed) {}
}
