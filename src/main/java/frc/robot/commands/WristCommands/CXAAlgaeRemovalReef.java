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
public class CXAAlgaeRemovalReef extends Command {
  private CXAMech cxaMech;
  private ElevatorMech elevatorMech;

  private Level level;

  private int counter;
  /** Creates a new CXAAlgaeRemovalReef. */
  public CXAAlgaeRemovalReef(CXAMech cxaMech, ElevatorMech elevatorMech, Level level) {
    this.cxaMech = cxaMech;
    this.elevatorMech = elevatorMech;
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cxaMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (level == Level.A1) {
      if (Math.abs(elevatorMech.getElevatorPosition() - ElevatorConstants.A1Pose / 2) <= 0.2) {
        cxaMech.setWristAlgaeRemovePosition();
      }
    } else {
      if (Math.abs(elevatorMech.getElevatorPosition() - ElevatorConstants.A2Pose / 2) <= 0.2) {
        cxaMech.setWristAlgaeRemovePosition();
      }
    }
    cxaMech.intakeAlgae();
    if (cxaMech.getCXAMotorCurrentAmps() >= 30) {
      counter++;
    }

    if (counter >= 25) {
      cxaMech.setAlgaeBool(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
