// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CXAMech extends SubsystemBase {
  private CXAMechIO io;
  private CXAMechIOInputsAutoLogged inputs = new CXAMechIOInputsAutoLogged();

  private boolean detectAlgae;

  private Alert cxaMotorDisconnected = new Alert("CXA Motor Disconnected", AlertType.kError);
  private Alert pivotMotorDisconnected =
      new Alert("Wrist Pivot Motor Disconnected", AlertType.kError);
  private Alert hopperMotorDisconnected = new Alert("Hopper Motor Disconnected", AlertType.kError);
  private Alert absEncDisconnected = new Alert("Wrist Abs Enc Disconnected", AlertType.kError);
  private Alert canRangeDisconnected = new Alert("Can Range Disconnected", AlertType.kError);

  /** Creates a new CXAMech. */
  public CXAMech(CXAMechIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("CXA_Mech", inputs);

    cxaMotorDisconnected.set(inputs.cxaMotorDiconnected);
    pivotMotorDisconnected.set(inputs.wristMotorDisconnected);
    hopperMotorDisconnected.set(inputs.hopperMotorDisconnected);
    absEncDisconnected.set(inputs.absEncDisconnected);
    canRangeDisconnected.set(inputs.canRangeDisconnected);
  }

  public void intakeAlgae() {
    io.setCXAMotorVel(CXAMechConstants.cxaMotorAlgaeRemovalVelocity, 1);
  }

  public void algaeHold() {
    io.setWristPosition(CXAMechConstants.algaeRestPosition);
    io.setCXAMotorVel(CXAMechConstants.cxaMotorAlgaeHoldVelocity, 0);
  }

  public void ejectCoral() {
    io.setCXAMotorVel(CXAMechConstants.coralEjectVelocity, 0);
  }

  public void ejectAlgae() {
    io.setCXAMotorVel(CXAMechConstants.algaeEjectVelocity, 1);
  }

  public void setWristAngle(double position) {
    io.setWristPosition(position);
  }

  public void setWristL1Angle() {
    io.setWristPosition(0);
  }

  public void setWristL2Angle() {
    io.setWristPosition(0);
  }

  public void setWristL3Angle() {
    io.setWristPosition(0);
  }

  public void setWristL4ngle() {
    io.setWristPosition(CXAMechConstants.l4WristPosition);
  }

  public void setWristAlgaeRemovePosition() {
    io.setWristPosition(CXAMechConstants.algaeRemovalWristPosition);
  }

  public void setWristAlgaePrimePosition() {
    io.setWristPosition(CXAMechConstants.algaeScorePrimePosition);
  }

  public void setWristAlgaeEndPosition() {
    io.setWristPosition(CXAMechConstants.algaeScoreEndPosition);
  }

  public void setWristFeederAngle() {
    io.setWristPosition(CXAMechConstants.feederPosition);
  }

  public void runHopperFeeder() {
    io.setHopperSpeed(CXAMechConstants.coralHopperSpeed);
  }

  public void stopCXAMotor() {
    io.setCXAMotorSpeed(0);
  }

  public void stopHopper() {
    io.setHopperSpeed(0);
  }

  public double getWristAngle() {
    return inputs.wristAngle;
  }

  public double getCXAMotorCurrentAmps() {
    return inputs.cxaMotorCurrentAmps;
  }

  public boolean isCoralDetected() {
    return inputs.detectCoral;
  }

  public boolean isThereCoral() {
    return io.isThereCoral();
  }

  public boolean doWeHaveAlgae() {
    return inputs.detectAlgae;
  }

  public void setAlgaeBool(boolean set) {
    inputs.detectAlgae = set;
  }
}
