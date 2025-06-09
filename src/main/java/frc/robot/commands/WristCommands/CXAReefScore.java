// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Level;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorMech;
import frc.robot.subsystems.Wrist.CXAMech;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CXAReefScore extends Command {
  private CXAMech cxaMech;
  private ElevatorMech elevatorMech;
  private Level level;
  private boolean readyToExecute;
  private int counter;
  private boolean isFinished;
  /** Creates a new SetWristAngle. */
  public CXAReefScore(
      CXAMech cxaMech, ElevatorMech elevatorMech, Level level, boolean readyToExecute) {
    this.cxaMech = cxaMech;
    this.elevatorMech = elevatorMech;
    this.level = level;
    this.readyToExecute = readyToExecute;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cxaMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (level) {
      default:
        if (Math.abs(elevatorMech.getElevatorPosition() - ElevatorConstants.L1Pose / 2) <= 0.1) {
          cxaMech.setWristL1Angle();
          if (readyToExecute) {
            cxaMech.ejectCoral();
            counter++;
          }
        }
        break;
      case L2:
        if (Math.abs(elevatorMech.getElevatorPosition() - ElevatorConstants.L2Pose / 2) <= 0.1) {
          cxaMech.setWristL2Angle();
          if (readyToExecute) {
            cxaMech.ejectCoral();
            counter++;
          }
        }
        break;
      case L3:
        if (Math.abs(elevatorMech.getElevatorPosition() - ElevatorConstants.L3Pose / 2) <= 0.1) {
          cxaMech.setWristL3Angle();
          if (readyToExecute) {
            cxaMech.ejectCoral();
            counter++;
          }
        }
        break;
      case L4:
        if (Math.abs(elevatorMech.getElevatorPosition() - ElevatorConstants.L4Pose / 2) <= 0.1) {
          cxaMech.setWristL4ngle();
          if (readyToExecute) {
            cxaMech.ejectCoral();
            counter++;
          }
        }
        break;
      case Net:
        if (Math.abs(elevatorMech.getElevatorPosition() - ElevatorConstants.NetPose / 2) <= 0.1) {
          if (readyToExecute) {
            cxaMech.ejectAlgae();
            cxaMech.setWristAlgaeEndPosition();
            counter++;
          } else {
            cxaMech.setWristAlgaePrimePosition();
          }
        }
        break;
      case AlgaeSpit:
        cxaMech.ejectAlgae();
        counter++;
    }

    if (counter >= 25) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
