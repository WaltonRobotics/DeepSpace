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
    final double maxVelocity = Units.feetToMeters(10);
    final double maxAccel = Units.feetToMeters(4);

    Pose2d startPose = new Pose2d(Units.feetToMeters(0), 0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(Units.feetToMeters(6), 0, Rotation2d.fromDegrees(0));

    ArrayList<Translation2d> waypoints = new ArrayList<>();

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
    final double maxVelocity = Units.feetToMeters(5);
    final double maxAccel = Units.feetToMeters(4);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(5.238), Units.feetToMeters(10.097), Rotation2d.fromDegrees(180)));
    waypoints.add(new Pose2d(Units.feetToMeters(17.7), Units.feetToMeters(7.883), Rotation2d.fromDegrees(165.983)));
    waypoints.add(new Pose2d(Units.feetToMeters(22.064), Units.feetToMeters(3.168), Rotation2d.fromDegrees(213)));

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
  public static Trajectory generateToRightCargo1() {

    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(6);
    final double maxAccel = Units.feetToMeters(3);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(5.282), Units.feetToMeters(9.821), Rotation2d.fromDegrees(0)));
    waypoints.add(new Pose2d(Units.feetToMeters(22.2), Units.feetToMeters(10.051), Rotation2d.fromDegrees(90)));

    ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
    centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(4)));

    return TrajectoryGenerator.generateTrajectory(
            waypoints,
            centripetalAccelerationConstraints,
            startVelocity,
            endVelocity,
            maxVelocity,
            maxAccel,
            false);
  }
}

