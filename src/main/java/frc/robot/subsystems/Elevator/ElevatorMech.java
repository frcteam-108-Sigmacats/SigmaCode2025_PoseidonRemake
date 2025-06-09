// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorMech extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  /** Creates a new ElevatorMech. */
  public ElevatorMech(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator_Mech", inputs);
    // This method will be called once per scheduler run
  }

  public void setRestPosition() {
    io.setElevatorPosition(0);
  }

  public void setL1Position() {
    io.setElevatorPosition(ElevatorConstants.L1Pose);
  }

  public void setL2Position() {
    io.setElevatorPosition(ElevatorConstants.L2Pose);
  }

  public void setL3Position() {
    io.setElevatorPosition(ElevatorConstants.L3Pose);
  }

  public void setL4Position() {
    io.setElevatorPosition(ElevatorConstants.L4Pose);
  }

  public void setA1Position() {
    io.setElevatorPosition(ElevatorConstants.A1Pose);
  }

  public void setA2Position() {
    io.setElevatorPosition(ElevatorConstants.A2Pose);
  }

  public void setNetPosition() {
    io.setElevatorPosition(ElevatorConstants.NetPose);
  }

  public double getElevatorPosition() {
    return inputs.elevatorPos;
  }
}
