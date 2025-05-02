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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class PoseConstants {

    public static final List<Pose2d> leftPoses =
        List.of(
            new Pose2d(new Translation2d(13.61, 2.76), Rotation2d.fromDegrees(120)), // 6
            new Pose2d(new Translation2d(14.43, 3.87), Rotation2d.fromDegrees(180)), // 7
            new Pose2d(new Translation2d(13.89, 5.15), Rotation2d.fromDegrees(-120)), // 8
            new Pose2d(new Translation2d(12.5, 5.3), Rotation2d.fromDegrees(-60)), // 9
            new Pose2d(new Translation2d(11.69, 4.19), Rotation2d.fromDegrees(0)), // 10
            new Pose2d(new Translation2d(12.24, 2.94), Rotation2d.fromDegrees(60)), // 11
            new Pose2d(new Translation2d(3.67, 2.92), Rotation2d.fromDegrees(60)), // 17
            new Pose2d(new Translation2d(3.115, 4.19), Rotation2d.fromDegrees(0)), // 18
            new Pose2d(new Translation2d(3.94, 5.29), Rotation2d.fromDegrees(-60)), // 19
            new Pose2d(new Translation2d(5.31, 5.14), Rotation2d.fromDegrees(-120)), // 20
            new Pose2d(new Translation2d(5.86, 3.87), Rotation2d.fromDegrees(180)), // 21
            new Pose2d(new Translation2d(5.04, 2.76), Rotation2d.fromDegrees(120))); // 22
    public static final List<Pose2d> rightPoses =
        List.of(
            new Pose2d(new Translation2d(13.90, 2.92), Rotation2d.fromDegrees(120)), // 6
            new Pose2d(new Translation2d(14.43, 4.20), Rotation2d.fromDegrees(180)), // 7
            new Pose2d(new Translation2d(13.59, 5.31), Rotation2d.fromDegrees(-120)), // 8
            new Pose2d(new Translation2d(12.21, 5.15), Rotation2d.fromDegrees(-60)), // 9
            new Pose2d(new Translation2d(11.69, 3.84), Rotation2d.fromDegrees(0)), // 10
            new Pose2d(new Translation2d(12.53, 2.74), Rotation2d.fromDegrees(60)), // 11
            new Pose2d(new Translation2d(3.96, 2.74), Rotation2d.fromDegrees(60)), // 17
            new Pose2d(new Translation2d(3.115, 3.86), Rotation2d.fromDegrees(0)), // 18
            new Pose2d(new Translation2d(3.63, 5.14), Rotation2d.fromDegrees(-60)), // 19
            new Pose2d(new Translation2d(5.01, 5.31), Rotation2d.fromDegrees(-120)), // 20
            new Pose2d(new Translation2d(5.86, 4.2), Rotation2d.fromDegrees(180)), // 21
            new Pose2d(new Translation2d(5.33, 2.92), Rotation2d.fromDegrees(120)) // 22
            );
    public static final List<Pose2d> humanStationPoses =
        (List<Pose2d>)
            List.of(
                new Pose2d(new Translation2d(1.297, 7.174), Rotation2d.fromDegrees(-53)),
                new Pose2d(new Translation2d(1.151, 0.963), Rotation2d.fromDegrees(53)),
                new Pose2d(new Translation2d(16.341, 0.944), Rotation2d.fromDegrees(127)),
                new Pose2d(new Translation2d(16.049, 7.330), Rotation2d.fromDegrees(-127)));

    public static final Map<Integer, Pose2d> algaeBluePoses =
        (Map<Integer, Pose2d>)
            Map.of(
                17,
                new Pose2d(new Translation2d(3.851, 2.923), Rotation2d.fromDegrees(60)),
                18,
                new Pose2d(new Translation2d(3.237, 4.025), Rotation2d.fromDegrees(0)),
                19,
                new Pose2d(new Translation2d(3.861, 5.107), Rotation2d.fromDegrees(-60)),
                20,
                new Pose2d(new Translation2d(5.25, 5.01), Rotation2d.fromDegrees(-120)),
                21,
                new Pose2d(new Translation2d(5.753, 4.015), Rotation2d.fromDegrees(180)),
                22,
                new Pose2d(new Translation2d(5.109, 2.943), Rotation2d.fromDegrees(120)));

    public static final Map<Integer, Pose2d> algaeRedPoses =
        (Map<Integer, Pose2d>)
            Map.of(
                6,
                new Pose2d(new Translation2d(13.670, 2.962), Rotation2d.fromDegrees(120)),
                7,
                new Pose2d(new Translation2d(14.313, 4.015), Rotation2d.fromDegrees(180)),
                8,
                new Pose2d(new Translation2d(13.689, 5.127), Rotation2d.fromDegrees(-120)),
                9,
                new Pose2d(new Translation2d(12.451, 5.107), Rotation2d.fromDegrees(-60)),
                10,
                new Pose2d(new Translation2d(11.797, 4.015), Rotation2d.fromDegrees(0)),
                11,
                new Pose2d(new Translation2d(12.451, 2.923), Rotation2d.fromDegrees(60)));
  }
}
