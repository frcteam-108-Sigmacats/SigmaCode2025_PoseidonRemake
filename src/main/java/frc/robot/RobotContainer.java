// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Level;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PoseConstants;
import frc.robot.commands.ClimberCommands.DeepClimb;
import frc.robot.commands.ClimberCommands.PrimeClimb;
import frc.robot.commands.CommandGroups.AlgaeRemoval;
import frc.robot.commands.CommandGroups.HumanStationFeeder;
import frc.robot.commands.CommandGroups.NetScore;
import frc.robot.commands.CommandGroups.ReefScore;
import frc.robot.commands.DriveCommands.DriveCommands;
import frc.robot.commands.DriveCommands.DriveToPose;
import frc.robot.commands.ElevatorCommands.ElevatorReefCommand;
import frc.robot.commands.ElevatorCommands.ElevatorRestCommand;
import frc.robot.commands.WristCommands.CXAHumanFeeder;
import frc.robot.commands.WristCommands.CXAReefScore;
import frc.robot.commands.WristCommands.CXARestCommand;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Climber.ClimberMech;
import frc.robot.subsystems.Elevator.ElevatorIOReal;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorMech;
import frc.robot.subsystems.Vision.VisionIOReal;
import frc.robot.subsystems.Vision.VisionMech;
import frc.robot.subsystems.Wrist.CXAMech;
import frc.robot.subsystems.Wrist.CXAMechIOReal;
import frc.robot.subsystems.Wrist.CXAMechIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMix;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private CXAMech cxaMech;

  private ElevatorMech elevatorMech;

  private ClimberMech climber;

  private final VisionMech vision;

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  private SwerveDriveSimulation driveSimulation;

  private boolean hasAlgae = false;

  private int counter = 200;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision = new VisionMech(new VisionIOReal());
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOMix(0),
                new ModuleIOMix(1),
                new ModuleIOMix(2),
                new ModuleIOMix(3),
                (pose) -> {},
                vision);
        cxaMech = new CXAMech(new CXAMechIOReal());
        elevatorMech = new ElevatorMech(new ElevatorIOReal());
        climber = new ClimberMech(new ClimberIOReal());
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        vision = new VisionMech(new VisionIOReal());
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()) {},
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose,
                vision);

        cxaMech = new CXAMech(new CXAMechIOSim(driveSimulation));

        elevatorMech = new ElevatorMech(new ElevatorIOSim());

        climber = new ClimberMech(new ClimberIOReal());
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = new VisionMech(new VisionIOReal());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {},
                vision);
        break;
    }

    makeAuto();

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    cxaMech.setDefaultCommand(new CXARestCommand(cxaMech));

    elevatorMech.setDefaultCommand(new ElevatorRestCommand(elevatorMech, cxaMech));

    controller
        .leftBumper()
        .and(() -> Constants.currentMode == Mode.SIM)
        .onTrue(
            Commands.runOnce(() -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())));
    controller.leftBumper().whileTrue(new DriveToPose(drive, true, "Reef", controller));
    controller
        .rightBumper()
        .and(() -> Constants.currentMode == Mode.SIM)
        .onTrue(
            Commands.runOnce(() -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())));
    controller.rightBumper().whileTrue(new DriveToPose(drive, false, "Reef", controller));

    controller.x().onTrue(Commands.runOnce(() -> hasAlgae = cxaMech.doWeHaveAlgae()));
    controller
        .x()
        .and(() -> !hasAlgae)
        .whileTrue(new ReefScore(elevatorMech, cxaMech, Level.L1, false));
    controller
        .x()
        .and(() -> hasAlgae)
        .whileTrue(new ReefScore(elevatorMech, cxaMech, Level.AlgaeSpit, false));
    controller
        .x()
        .and(() -> !hasAlgae)
        .whileFalse(new ReefScore(elevatorMech, cxaMech, Level.L1, true));
    controller
        .x()
        .and(() -> hasAlgae)
        .whileFalse(new ReefScore(elevatorMech, cxaMech, Level.AlgaeSpit, true));
    controller
        .x()
        .and(() -> !hasAlgae)
        .whileFalse(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.6
                                        + Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofRelativeUnits(
                                    3.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofBaseUnits(
                                    cxaMech.getWristAngle() - 20,
                                    edu.wpi.first.units.Units.Degrees)))));
    controller
        .x()
        .and(() -> hasAlgae)
        .whileFalse(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeAlgaeOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.8
                                        + (Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2)),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    1.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    -cxaMech.getWristAngle() + 20,
                                    edu.wpi.first.units.Units.Degrees)))));
    controller.a().onTrue(Commands.runOnce(() -> hasAlgae = cxaMech.doWeHaveAlgae()));
    controller
        .a()
        .and(() -> !hasAlgae)
        .whileTrue(new ReefScore(elevatorMech, cxaMech, Level.L2, false));
    controller
        .a()
        .and(() -> hasAlgae)
        .whileTrue(new ReefScore(elevatorMech, cxaMech, Level.AlgaeSpit, false));
    controller
        .a()
        .and(() -> !hasAlgae)
        .whileFalse(new ReefScore(elevatorMech, cxaMech, Level.L2, true));
    controller
        .a()
        .and(() -> hasAlgae)
        .whileFalse(new ReefScore(elevatorMech, cxaMech, Level.AlgaeSpit, true));
    controller
        .a()
        .and(() -> !hasAlgae)
        .whileFalse(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.6
                                        + Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    3.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    cxaMech.getWristAngle() - 20,
                                    edu.wpi.first.units.Units.Degrees)))));

    controller
        .a()
        .and(() -> hasAlgae)
        .whileFalse(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeAlgaeOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.8
                                        + (Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2)),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    1.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    -cxaMech.getWristAngle() + 20,
                                    edu.wpi.first.units.Units.Degrees)))));

    controller.b().onTrue(Commands.runOnce(() -> hasAlgae = cxaMech.doWeHaveAlgae()));
    controller
        .b()
        .and(() -> !hasAlgae)
        .whileTrue(new ReefScore(elevatorMech, cxaMech, Level.L3, false));
    controller
        .b()
        .and(() -> hasAlgae)
        .whileTrue(new ReefScore(elevatorMech, cxaMech, Level.AlgaeSpit, false));
    controller
        .b()
        .and(() -> !hasAlgae)
        .whileFalse(new ReefScore(elevatorMech, cxaMech, Level.L3, true));
    controller
        .b()
        .and(() -> hasAlgae)
        .whileFalse(new ReefScore(elevatorMech, cxaMech, Level.AlgaeSpit, true));
    controller
        .b()
        .and(() -> !hasAlgae)
        .whileFalse(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.6
                                        + Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    3.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    cxaMech.getWristAngle() - 20,
                                    edu.wpi.first.units.Units.Degrees)))));

    controller
        .b()
        .and(() -> hasAlgae)
        .whileFalse(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeAlgaeOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.8
                                        + (Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2)),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    1.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    -cxaMech.getWristAngle() + 20,
                                    edu.wpi.first.units.Units.Degrees)))));

    controller.y().onTrue(Commands.runOnce(() -> hasAlgae = cxaMech.doWeHaveAlgae()));
    controller
        .y()
        .and(() -> !hasAlgae)
        .whileTrue(new ReefScore(elevatorMech, cxaMech, Level.L4, false));

    controller
        .y()
        .and(() -> hasAlgae)
        .whileTrue(
            new NetScore(drive, false, controller, elevatorMech, cxaMech, Level.Net, false)
                .andThen(new ReefScore(elevatorMech, cxaMech, Level.Net, false)));

    controller
        .y()
        .and(() -> !hasAlgae)
        .whileFalse(new ReefScore(elevatorMech, cxaMech, Level.L4, true));
    controller
        .y()
        .and(() -> hasAlgae)
        .whileFalse(
            new NetScore(drive, false, controller, elevatorMech, cxaMech, Level.Net, true)
                .andThen(new ReefScore(elevatorMech, cxaMech, Level.Net, true)));
    controller
        .y()
        .and(() -> !hasAlgae)
        .whileFalse(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.6
                                        + (Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2)),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    3.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    -cxaMech.getWristAngle() - 20,
                                    edu.wpi.first.units.Units.Degrees)))));

    controller
        .y()
        .and(() -> hasAlgae)
        .whileFalse(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeAlgaeOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.8
                                        + (Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2)),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    3.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    -cxaMech.getWristAngle() + 20,
                                    edu.wpi.first.units.Units.Degrees)))));
    controller.leftTrigger().toggleOnTrue(new HumanStationFeeder(cxaMech, drive, controller));

    controller
        .leftStick()
        .whileTrue(
            new DriveToPose(drive, false, "AlgaeRemoval", controller)
                .withDeadline(new AlgaeRemoval(cxaMech, elevatorMech, Level.A1))
                .andThen(new AlgaeRemoval(cxaMech, elevatorMech, Level.A1)));

    controller
        .rightStick()
        .whileTrue(
            new DriveToPose(drive, false, "AlgaeRemoval", controller)
                .withDeadline(new AlgaeRemoval(cxaMech, elevatorMech, Level.A2))
                .andThen(new AlgaeRemoval(cxaMech, elevatorMech, Level.A2)));
    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            ReefscapeCoralOnFly.DropFromCoralStation(
                                CoralStationsSide.LEFT_STATION,
                                DriverStation.getAlliance().get(),
                                false))));
    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            ReefscapeCoralOnFly.DropFromCoralStation(
                                CoralStationsSide.RIGHT_STATION,
                                DriverStation.getAlliance().get(),
                                false))));
    controller
        .povUp()
        .and(() -> (Constants.currentMode == Mode.REAL))
        .toggleOnTrue(new PrimeClimb(climber, drive));
    controller
        .povDown()
        .and(() -> (Constants.currentMode == Mode.REAL))
        .whileTrue(new DeepClimb(climber));
  }

  public void makeAuto() {
    NamedCommands.registerCommand(
        "LeftAutoAlign", new DriveToPose(drive, true, "Reef", controller));
    NamedCommands.registerCommand(
        "RightAutoAlign", new DriveToPose(drive, false, "Reef", controller));
    NamedCommands.registerCommand(
        "HumanFeeder",
        new InstantCommand(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePiece(
                            new ReefscapeCoralOnField(
                                driveSimulation
                                    .getSimulatedDriveTrainPose()
                                    .nearest(Constants.PoseConstants.humanStationPoses))))
            .andThen(new CXAHumanFeeder(cxaMech)));
    NamedCommands.registerCommand(
        "L4Position", new ReefScore(elevatorMech, cxaMech, Level.L4, false));
    NamedCommands.registerCommand(
        "L4Score",
        new ReefScore(elevatorMech, cxaMech, Level.L4, true)
            .finallyDo(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.6
                                        + (Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2)),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    3.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    -cxaMech.getWristAngle() - 20,
                                    edu.wpi.first.units.Units.Degrees)))));
    NamedCommands.registerCommand(
        "NetPosition",
        new CXAReefScore(cxaMech, elevatorMech, Level.Net, false)
            .alongWith(new ElevatorReefCommand(elevatorMech, Level.Net)));
    NamedCommands.registerCommand(
        "NetScore",
        new CXAReefScore(cxaMech, elevatorMech, Level.Net, true)
            .raceWith(new ElevatorReefCommand(elevatorMech, Level.Net))
            .finallyDo(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeAlgaeOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.126, 0.0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Distance.ofRelativeUnits(
                                    0.8
                                        + (Units.feetToMeters(
                                            elevatorMech.getElevatorPosition() * 2)),
                                    edu.wpi.first.units.Units.Meters),
                                LinearVelocity.ofBaseUnits(
                                    3.0, edu.wpi.first.units.Units.FeetPerSecond),
                                Angle.ofRelativeUnits(
                                    -cxaMech.getWristAngle() + 20,
                                    edu.wpi.first.units.Units.Degrees)))));
    NamedCommands.registerCommand(
        "AlgaeRemoveHigh",
        new AlgaeRemoval(cxaMech, elevatorMech, Level.A2)
            .alongWith(new DriveToPose(drive, false, "AlgaeRemoval", controller))
            .withTimeout(4));
    NamedCommands.registerCommand(
        "AlgaeRemoveLow",
        new AlgaeRemoval(cxaMech, elevatorMech, Level.A1)
            .alongWith(new DriveToPose(drive, false, "AlgaeRemoval", controller))
            .withTimeout(4));
    NamedCommands.registerCommand("RestElevator", new ElevatorRestCommand(elevatorMech, cxaMech));
    NamedCommands.registerCommand("RestWrist", new CXARestCommand(cxaMech));
    NamedCommands.registerCommand(
        "StartPoseResetC",
        new ConditionalCommand(
            new InstantCommand(
                () ->
                    drive.resetOdometry(
                        new Pose2d(7.588, 6.333, Rotation2d.fromDegrees(-158.068)))),
            new InstantCommand(
                () -> drive.resetOdometry(new Pose2d(10.214, 1.765, Rotation2d.fromDegrees(30)))),
            () -> DriverStation.getAlliance().get() == Alliance.Blue));
    NamedCommands.registerCommand(
        "StartPoseResetP",
        new ConditionalCommand(
            new InstantCommand(
                () ->
                    drive.resetOdometry(new Pose2d(7.588, 1.370, Rotation2d.fromDegrees(158.068)))),
            new InstantCommand(
                () -> drive.resetOdometry(new Pose2d(10.214, 6.572, Rotation2d.fromDegrees(-30)))),
            () -> DriverStation.getAlliance().get() == Alliance.Blue));
    NamedCommands.registerCommand(
        "StartPoseResetM",
        new ConditionalCommand(
            new InstantCommand(
                () -> drive.resetOdometry(new Pose2d(7.588, 3.935, Rotation2d.fromDegrees(180)))),
            new InstantCommand(
                () -> drive.resetOdometry(new Pose2d(10.214, 3.935, Rotation2d.fromDegrees(0)))),
            () -> DriverStation.getAlliance().get() == Alliance.Blue));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    counter++;

    SimulatedArena.getInstance().simulationPeriodic();

    Pose2d targetPose =
        driveSimulation.getSimulatedDriveTrainPose().nearest(PoseConstants.humanStationPoses);
    double x = driveSimulation.getSimulatedDriveTrainPose().getX() - targetPose.getX();
    double y = driveSimulation.getSimulatedDriveTrainPose().getY() - targetPose.getY();
    double distance = Math.hypot(x, y);
    if (Math.abs(distance) <= 2) {
      if (targetPose.getY() > 3) {
        if (counter >= 200) {
          SimulatedArena.getInstance()
              .addGamePieceProjectile(
                  ReefscapeCoralOnFly.DropFromCoralStation(
                      DriverStation.getAlliance().get() == Alliance.Blue
                          ? CoralStationsSide.LEFT_STATION
                          : CoralStationsSide.RIGHT_STATION,
                      DriverStation.getAlliance().get(),
                      false));
          counter = 0;
        }
      } else {
        if (counter >= 200) {
          SimulatedArena.getInstance()
              .addGamePieceProjectile(
                  ReefscapeCoralOnFly.DropFromCoralStation(
                      DriverStation.getAlliance().get() == Alliance.Blue
                          ? CoralStationsSide.RIGHT_STATION
                          : CoralStationsSide.LEFT_STATION,
                      DriverStation.getAlliance().get(),
                      false));
          counter = 0;
        }
      }
    }

    Pose3d elevatorBasePose = new Pose3d(0, 0, 0.01, new Rotation3d());
    Pose3d secondStageElevatorPose =
        new Pose3d(
            0,
            0,
            0.05 + (Units.feetToMeters(elevatorMech.getElevatorPosition())),
            new Rotation3d());
    Pose3d wristCarriagePose =
        new Pose3d(
            0,
            0,
            0.08 + (Units.feetToMeters(elevatorMech.getElevatorPosition()) * 2),
            new Rotation3d());
    Pose3d wristPose =
        new Pose3d(
            0.126,
            0.0,
            0.35 + (Units.feetToMeters(elevatorMech.getElevatorPosition()) * 2),
            new Rotation3d(0.0, Units.degreesToRadians(cxaMech.getWristAngle()), 0.0));
    Pose3d hopperPose = new Pose3d(-0.05, -0.02, 0.61, new Rotation3d());

    // this.coralScoreSim
    //     .getPose3d()
    //     .transformBy(
    //         new Transform3d(
    //             0, 0, wristPose.getZ(), new Rotation3d(0, wristPose.getRotation().getY(), 0)));

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          elevatorBasePose, secondStageElevatorPose, wristCarriagePose, wristPose, hopperPose
        });
  }
}
