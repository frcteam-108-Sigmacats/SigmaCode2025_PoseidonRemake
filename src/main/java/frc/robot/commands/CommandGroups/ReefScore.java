// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.Level;
import frc.robot.commands.ElevatorCommands.ElevatorReefCommand;
import frc.robot.commands.WristCommands.CXAReefScore;
import frc.robot.subsystems.Elevator.ElevatorMech;
import frc.robot.subsystems.Wrist.CXAMech;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReefScore extends ParallelRaceGroup {
  /** Creates a new ReefScore. */
  public ReefScore(ElevatorMech elevatorMech, CXAMech cxaMech, Level l1, boolean readyToExecute) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ElevatorReefCommand(elevatorMech, l1),
        new CXAReefScore(cxaMech, elevatorMech, l1, readyToExecute));
  }
}
