package frc.robot;

import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Geometry.Translation2d;
import lib.Utils.Units;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryGenerator;
import lib.trajectory.constraint.CentripetalAccelerationConstraint;
import lib.trajectory.constraint.TrajectoryConstraint;

import java.util.ArrayList;

public class Paths {

  public static Trajectory generateTestTrajectory() {
    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(12);
    final double maxAccel = Units.feetToMeters(12);

    Pose2d startPose = new Pose2d(Units.feetToMeters(0), 0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(Units.feetToMeters(10), 0, Rotation2d.fromDegrees(0));

    ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();

    return TrajectoryGenerator.generateTrajectory(
            startPose,
            waypoints,
            endPose,
            new ArrayList<>(),
            startVelocity,
            endVelocity,
            maxVelocity,
            maxAccel,
            false
    );
  }

  public static Trajectory generateToRightRocket1() {

    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(12);
    final double maxAccel = Units.feetToMeters(12);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(5.018), Units.feetToMeters(11.337), Rotation2d.fromDegrees(179.658)));
    waypoints.add(new Pose2d(Units.feetToMeters(12.731), Units.feetToMeters(8.352), Rotation2d.fromDegrees(155.447)));
    waypoints.add(new Pose2d(Units.feetToMeters(22.932), Units.feetToMeters(4.776), Rotation2d.fromDegrees(-145.804)));

    ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
    centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(4)));

    return TrajectoryGenerator.generateTrajectory(

    waypoints,
    centripetalAccelerationConstraints,
    startVelocity,
    endVelocity,
    maxVelocity,
    maxAccel,
    true);
  }
}

