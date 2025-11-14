// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class ClimberMech extends SubsystemBase {
  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private Alert climberPivotMotorAlert =
      new Alert("Climber Pivot Motor Disconnected", AlertType.kError);
  private Alert climberIntakeMotorAlert =
      new Alert("Climber Intake Motor Disconnected", AlertType.kError);
  /** Creates a new ClimberMech. */
  public ClimberMech(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber_Mech", inputs);
    climberIntakeMotorAlert.set(
        inputs.climberIntakeMotorDisconnected && Constants.currentMode == Mode.REAL);
    climberPivotMotorAlert.set(
        inputs.climberPivotMotorDisconnected && Constants.currentMode == Mode.REAL);
    // This method will be called once per scheduler run
  }

  public void unHingeServo() {
    io.setServoPosition(0.5);
  }

  public void hingeServo() {
    io.setServoPosition(0);
  }

  public void setClimberOut() {
    io.setClimberPosition(ClimberConstants.climberOutPosition);
  }

  public void setClimberIn() {
    io.setClimberPosition(ClimberConstants.climberInPosition);
  }

  public void setClimberForwardSpeed() {
    io.setClimberPivotSpeed(ClimberConstants.climberOutSpeed);
  }

  public void setClimberReverseSpeed() {
    io.setClimberPivotSpeed(ClimberConstants.climberInSpeed);
  }

  public void stopClimberPivotMotor() {
    io.setClimberPivotSpeed(0);
  }

  public void intakeCage() {
    io.setClimberIntakeSpeed(ClimberConstants.cageIntakeSpeed);
  }

  public void outtakeCage() {
    io.setClimberIntakeSpeed(ClimberConstants.cageOuttakeSpeed);
  }

  public void stopClimberIntakeMotor() {
    io.setClimberIntakeSpeed(0);
  }

  public double getClimberPos() {
    return inputs.climberAngle;
  }

  public boolean getCageReader() {
    return inputs.cageReader;
  }
}
