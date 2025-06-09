// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Level;
import frc.robot.subsystems.Elevator.ElevatorMech;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorReefCommand extends Command {
  private ElevatorMech elevatorMech;
  private Level level;
  /** Creates a new SetElevatorPosition. */
  public ElevatorReefCommand(ElevatorMech elevatorMech, Level level) {
    this.elevatorMech = elevatorMech;
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (level) {
      default:
        elevatorMech.setL1Position();
        break;
      case L2:
        elevatorMech.setL2Position();
        break;
      case L3:
        elevatorMech.setL3Position();
        break;
      case L4:
        elevatorMech.setL4Position();
        break;
      case A1:
        elevatorMech.setA1Position();
        break;
      case A2:
        elevatorMech.setA2Position();
        break;
      case Net:
        elevatorMech.setNetPosition();
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
