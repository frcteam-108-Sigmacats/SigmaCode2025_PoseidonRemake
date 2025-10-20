// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberMech;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrimeClimb extends Command {
  private ClimberMech climberMech;
  private Drive swerveSub;
  private CommandXboxController driverController;
  private int counter;
  private int climbStates;
  /** Creates a new PrimeClimb. */
  public PrimeClimb(ClimberMech climberMech, Drive swerveSub) {
    this.climberMech = climberMech;
    this.swerveSub = swerveSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    climbStates = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (climbStates) {
      case 0:
        swerveSub.setSlowSpeedBool(true);
        counter++;
        climberMech.unHingeServo();
        if (counter >= 15) {
          climbStates = 1;
        }
        break;
      case 1:
        counter = 0;
        climberMech.setClimberOut();
        if (climberMech.getClimberPos() >= (ClimberConstants.climberOutPosition - 3)
            && climberMech.getClimberPos() <= 350) {
          climbStates = 2;
        }
        break;
      case 2:
        counter++;
        climberMech.hingeServo();
        climberMech.stopClimberPivotMotor();
        climberMech.intakeCage();
        if (climberMech.getCageReader() && counter >= 15) {
          climbStates = 3;
        }
        break;
      case 3:
        climberMech.intakeCage();
        climberMech.setClimberIn();
        if (climberMech.getClimberPos() <= (ClimberConstants.climberInPosition + 3)) {
          climbStates = 4;
        }
        break;
      case 4:
        climberMech.stopClimberIntakeMotor();
        if (driverController.povDown().getAsBoolean()) {
          climberMech.setClimberReverseSpeed();
        } else {
          climberMech.stopClimberPivotMotor();
        }
        break;
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
