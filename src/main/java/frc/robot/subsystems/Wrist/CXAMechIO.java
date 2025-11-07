package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface CXAMechIO {
  @AutoLog
  public static class CXAMechIOInputs {
    public boolean cxaMotorDiconnected = false;
    public boolean wristMotorDisconnected = false;
    public boolean hopperMotorDisconnected = false;
    public boolean absEncDisconnected = false;
    public boolean canRangeDisconnected = false;
    public double cxaMotorVoltage = 0.0;
    public double cxaMotorCurrentAmps = 0.0;
    public double cxaWristVoltage = 0.0;
    public double cxaWristCurrentAmps = 0.0;
    public double hopperMotorVoltage = 0.0;
    public double hopperMotorCurrentAmps = 0.0;
    public double wristAngle = 0.0;
    public boolean detectCoral = false;
    public boolean detectAlgae = false;
  }

  public default void setCXAMotorVel(double velocity, int slot) {}

  public default void setWristPosition(double position) {}

  public default void setHopperSpeed(double speed) {}

  public default void setWristSpeed(double speed) {}

  public default void setCXAMotorSpeed(double speed) {}

  public default boolean isThereCoral() {
    return false;
  }

  public default void updateInputs(CXAMechIOInputs inputs) {}
}
