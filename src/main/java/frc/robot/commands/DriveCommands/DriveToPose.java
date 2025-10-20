// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  private Drive swerve;
  private PIDController translationController = new PIDController(0.657, 0.0, 0.0);
  private PIDController rotationController = new PIDController(0.025, 0, 0.002);
  private Pose2d targetPose;
  private boolean left;
  private double x, y;
  private String mode;
  private Translation2d translation;
  private double rotation;
  private CommandXboxController driver;
  /** Creates a new DriveToPose. */
  public DriveToPose(Drive swerve, boolean left, String mode, CommandXboxController driver) {
    this.swerve = swerve;
    this.left = left;
    this.mode = mode;
    this.driver = driver;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translation = new Translation2d();
    switch (mode) {
      default:
        if (left) {
          targetPose = swerve.getPose().nearest(Constants.PoseConstants.leftPoses);
        } else {

          targetPose = swerve.getPose().nearest(Constants.PoseConstants.rightPoses);
        }
        break;
      case ("Net"):
        targetPose =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? new Pose2d(7.72, swerve.getPose().getY(), Rotation2d.fromDegrees(0))
                : new Pose2d(9.890, swerve.getPose().getY(), Rotation2d.fromDegrees(0));
        break;
      case ("Feeder"):
        targetPose = swerve.getPose().nearest(Constants.PoseConstants.humanStationPoses);
        break;
      case ("AlgaeRemoval"):
        targetPose = swerve.getPose().nearest(PoseConstants.algaeReefPoses);
        break;
    }

    rotation = 0;
    translationController.enableContinuousInput(-5, 5);
    rotationController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds swerveSpeed = new ChassisSpeeds();
    switch (mode) {
      default:
        x = swerve.getPose().getX() - targetPose.getX();
        y = swerve.getPose().getY() - targetPose.getY();
        translation =
            new Translation2d(
                translationController.calculate(swerve.getPose().getX(), targetPose.getX()),
                translationController.calculate(swerve.getPose().getY(), targetPose.getY()));
        rotation =
            rotationController.calculate(
                swerve.getPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees());
        break;
      case ("Net"):
        x = swerve.getPose().getX() - targetPose.getX();
        y = swerve.getPose().getY() - targetPose.getY();
        translation =
            new Translation2d(
                translationController.calculate(swerve.getPose().getX(), targetPose.getX()),
                translationController.calculate(swerve.getPose().getY(), targetPose.getY()));
        rotation =
            rotationController.calculate(
                swerve.getPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees());
        break;
      case ("Feeder"):
        targetPose = swerve.getPose().nearest(Constants.PoseConstants.humanStationPoses);
        translation = new Translation2d(-driver.getLeftY(), -driver.getLeftX());
        rotation =
            rotationController.calculate(
                swerve.getPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees());
        break;
      case ("AlgaeRemoval"):
        translation =
            new Translation2d(
                translationController.calculate(swerve.getPose().getX(), targetPose.getX()),
                translationController.calculate(swerve.getPose().getY(), targetPose.getY()));
        rotation =
            rotationController.calculate(
                swerve.getPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees());
    }
    swerveSpeed =
        new ChassisSpeeds(
            translation.getX() * swerve.getMaxLinearSpeedMetersPerSec(),
            translation.getY() * swerve.getMaxLinearSpeedMetersPerSec(),
            rotation * swerve.getMaxAngularSpeedRadPerSec());
    boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    swerve.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            swerveSpeed,
            isFlipped ? swerve.getRotation().plus(new Rotation2d(180)) : swerve.getRotation()));
    Logger.recordOutput("/DriveToPosePID", translation);
    Logger.recordOutput("/DrivePoseChassisSpeeds", swerveSpeed);
    Logger.recordOutput("/TargetPose", targetPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.runVelocity(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mode == "Reef" || mode == "Net") {
      if (Math.abs(Math.hypot(x, y)) < 0.02
          && Math.abs(swerve.getRotation().getDegrees() - targetPose.getRotation().getDegrees())
              <= 0.5) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}
