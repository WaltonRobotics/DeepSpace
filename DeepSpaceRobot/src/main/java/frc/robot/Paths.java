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
    final double maxVelocity = Units.feetToMeters(4);
    final double maxAccel = Units.feetToMeters(4);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(5.326), Units.feetToMeters(9.867), Rotation2d.fromDegrees(0)));
    waypoints.add(new Pose2d(Units.feetToMeters(19.547), Units.feetToMeters(8.031), Rotation2d.fromDegrees(-5.803)));
    waypoints.add(new Pose2d(Units.feetToMeters(22.361), Units.feetToMeters(2.755), Rotation2d.fromDegrees(-146.059)));

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

  public static Trajectory generateRightBackUp() {
    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(4);
    final double maxAccel = Units.feetToMeters(3);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(20.453), Units.feetToMeters(1.694), Rotation2d.fromDegrees(-144.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(23.593), Units.feetToMeters(3.255), Rotation2d.fromDegrees(180.0)));

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

  public static Trajectory generateFromRightBackToLoading() {

    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(4);
    final double maxAccel = Units.feetToMeters(3);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(23.593), Units.feetToMeters(3.255), Rotation2d.fromDegrees(180)));
    waypoints.add(new Pose2d(Units.feetToMeters(16.425), Units.feetToMeters(4.77), Rotation2d.fromDegrees(178.482)));
    waypoints.add(new Pose2d(Units.feetToMeters(2.792), Units.feetToMeters(2.204), Rotation2d.fromDegrees(180)));

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

  public static Trajectory generateRightRocketLoadingBackUp() {
    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(4);
    final double maxAccel = Units.feetToMeters(3);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(1.412), Units.feetToMeters(2.199), Rotation2d.fromDegrees(180.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(14.622), Units.feetToMeters(2.75), Rotation2d.fromDegrees(180.0)));

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

  public static Trajectory generateRightToHab1() {
    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(4);
    final double maxAccel = Units.feetToMeters(3);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(5.37), Units.feetToMeters(9.867), Rotation2d.fromDegrees(0)));
    waypoints.add(new Pose2d(Units.feetToMeters(10.312), Units.feetToMeters(9.821), Rotation2d.fromDegrees(0)));
    waypoints.add(new Pose2d(Units.feetToMeters(16.424), Units.feetToMeters(7.163), Rotation2d.fromDegrees(0)));
    waypoints.add(new Pose2d(Units.feetToMeters(21.789), Units.feetToMeters(9.872), Rotation2d.fromDegrees(90)));
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

  public static Trajectory generateRightHab1Backup() {
    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(4);
    final double maxAccel = Units.feetToMeters(3);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(21.614), Units.feetToMeters(9.592), Rotation2d.fromDegrees(90.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(23.416), Units.feetToMeters(5.189), Rotation2d.fromDegrees(180.0)));

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

  public static Trajectory generateRightHab1ToLoadingStation() {

    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(4);
    final double maxAccel = Units.feetToMeters(3);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(23.416), Units.feetToMeters(5.189), Rotation2d.fromDegrees(180.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(12.291), Units.feetToMeters(4.592), Rotation2d.fromDegrees(180.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(3.012), Units.feetToMeters(2.296), Rotation2d.fromDegrees(180.0)));

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

  public static Trajectory generateRightHab1ToRocketBackup() {
    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(4);
    final double maxAccel = Units.feetToMeters(3);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(1.412), Units.feetToMeters(2.199), Rotation2d.fromDegrees(180.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(14.622), Units.feetToMeters(2.75), Rotation2d.fromDegrees(180.0)));

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

  public static Trajectory generateToFrontRightRocket() {

    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(5);
    final double maxAccel = Units.feetToMeters(4);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(5.282), Units.feetToMeters(9.821), Rotation2d.fromDegrees(0)));
    waypoints.add(new Pose2d(Units.feetToMeters(14.447), Units.feetToMeters(3.577), Rotation2d.fromDegrees(-30.0)));
    //waypoints.add(new Pose2d(Units.feetToMeters(14.182), Units.feetToMeters(2.847), Rotation2d.fromDegrees(-31.0)));

    ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
    centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(3)));

    return TrajectoryGenerator.generateTrajectory(
            waypoints,
            centripetalAccelerationConstraints,
            startVelocity,
            endVelocity,
            maxVelocity,
            maxAccel,
            false);
  }

  public static Trajectory generateBackupFrontRightRocket() {

    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(5);
    final double maxAccel = Units.feetToMeters(4);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(16.275), Units.feetToMeters(2.474), Rotation2d.fromDegrees(-30.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(12.688), Units.feetToMeters(6.057), Rotation2d.fromDegrees(-90.0)));
    //waypoints.add(new Pose2d(Units.feetToMeters(14.182), Units.feetToMeters(2.847), Rotation2d.fromDegrees(-31.0)));

    ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
    centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(3)));

    return TrajectoryGenerator.generateTrajectory(
            waypoints,
            centripetalAccelerationConstraints,
            startVelocity,
            endVelocity,
            maxVelocity,
            maxAccel,
            true);
  }

  public static Trajectory generateFrontRightRocketToLoadingStation() {

    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(5);
    final double maxAccel = Units.feetToMeters(4);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(12.688), Units.feetToMeters(6.057), Rotation2d.fromDegrees(-90.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(4.433), Units.feetToMeters(2.016), Rotation2d.fromDegrees(180)));
    //waypoints.add(new Pose2d(Units.feetToMeters(14.182), Units.feetToMeters(2.847), Rotation2d.fromDegrees(-31.0)));

    ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
    centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(3)));

    return TrajectoryGenerator.generateTrajectory(
            waypoints,
            centripetalAccelerationConstraints,
            startVelocity,
            endVelocity,
            maxVelocity,
            maxAccel,
            false);
  }

  public static Trajectory generateFrontRightRocketToSecondHatch() {

    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = Units.feetToMeters(5);
    final double maxAccel = Units.feetToMeters(4);

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(1.475), Units.feetToMeters(2.154), Rotation2d.fromDegrees(-180.0)));
    waypoints.add(new Pose2d(Units.feetToMeters(12.217), Units.feetToMeters(4.633), Rotation2d.fromDegrees(-39)));
    //waypoints.add(new Pose2d(Units.feetToMeters(14.182), Units.feetToMeters(2.847), Rotation2d.fromDegrees(-31.0)));

    ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
    centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(3)));

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

