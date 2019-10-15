package frc.robot;

import kotlin.Unit;
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

    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(-4, 0, Rotation2d.fromDegrees(0));

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

//    ArrayList<Pose2d> waypoints = new ArrayList<>();
//    waypoints.add(new Pose2d(Units.feetToMeters(5.067), Units.feetToMeters(11.395), Rotation2d.fromDegrees(0)));
//    waypoints.add(new Pose2d(Units.feetToMeters(17.163), Units.feetToMeters(7.745), Rotation2d.fromDegrees(-26.311)));
//    waypoints.add(new Pose2d(Units.feetToMeters(23.277), Units.feetToMeters(7.745), Rotation2d.fromDegrees(-26.311)));
//
//    ArrayList<CentripetalAccelerationConstraint> centripetalAccelerationConstraints = new ArrayList<>();
//    centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(4)));
//
//    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//
//    waypoints,
//    centripetalAccelerationConstraints,

      return null;
  }
}

