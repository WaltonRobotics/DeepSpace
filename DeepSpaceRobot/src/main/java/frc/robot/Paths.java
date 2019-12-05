package frc.robot;

import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.utils.Units;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryGenerator;
import lib.trajectory.constraint.CentripetalAccelerationConstraint;
import lib.trajectory.constraint.TrajectoryConstraint;

import java.util.ArrayList;

/**
 * Contains all paths for auto. Reflect across y = 13.5 to reverse
 */

public class Paths {

  /**
   * Contains paths to test trajectories
   */

  public static class TestTrajectories {

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
  }

  /**
   * Contains paths to do lv 1 back and front of right rocket
   */

  public static class RightRocketBackAndFrontLv1 {


      public static Trajectory generateToFrontLv1() {

          final double startVelocity = 0;
          final double endVelocity = 0.5;
          final double maxVelocity = Units.feetToMeters(12);
          final double maxAccel = Units.feetToMeters(5);

          ArrayList<Pose2d> waypoints = new ArrayList<>();
          waypoints.add(new Pose2d(Units.feetToMeters(5.282), Units.feetToMeters(9.821), Rotation2d.fromDegrees(0)));
          waypoints.add(new Pose2d(Units.feetToMeters(14.447), Units.feetToMeters(3.577), Rotation2d.fromDegrees(-30.0)));

          ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
          centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(5)));

          return TrajectoryGenerator.generateTrajectory(
                  waypoints,
                  centripetalAccelerationConstraints,
                  startVelocity,
                  endVelocity,
                  maxVelocity,
                  maxAccel,
                  false);
      }

      public static Trajectory generateBackUpFrontLv1() {

          final double startVelocity = 0;
          final double endVelocity = 0;
          final double maxVelocity = Units.feetToMeters(12);
          final double maxAccel = Units.feetToMeters(7);

          ArrayList<Pose2d> waypoints = new ArrayList<>();
          waypoints.add(new Pose2d(Units.feetToMeters(16.275), Units.feetToMeters(2.474), Rotation2d.fromDegrees(-30.0)));
          waypoints.add(new Pose2d(Units.feetToMeters(12.688), Units.feetToMeters(6.057), Rotation2d.fromDegrees(-90.0)));
          //waypoints.add(new Pose2d(Units.feetToMeters(14.182), Units.feetToMeters(2.847), Rotation2d.fromDegrees(-31.0)));

          ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
          centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(6)));

          return TrajectoryGenerator.generateTrajectory(
                  waypoints,
                  centripetalAccelerationConstraints,
                  startVelocity,
                  endVelocity,
                  maxVelocity,
                  maxAccel,
                  true);
      }

      public static Trajectory generateToLoadingStation() {

          final double startVelocity = 0;
          final double endVelocity = 0;
          final double maxVelocity = Units.feetToMeters(13);
          final double maxAccel = Units.feetToMeters(7);

          ArrayList<Pose2d> waypoints = new ArrayList<>();
          waypoints.add(new Pose2d(Units.feetToMeters(12.688), Units.feetToMeters(6.057), Rotation2d.fromDegrees(-90.0)));
          waypoints.add(new Pose2d(Units.feetToMeters(9.169), Units.feetToMeters(1.791), Rotation2d.fromDegrees(-180.0)));
          waypoints.add(new Pose2d(Units.feetToMeters(3.232), Units.feetToMeters(1.01), Rotation2d.fromDegrees(180.0)));

          ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
          centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(6)));

          return TrajectoryGenerator.generateTrajectory(
                  waypoints,
                  centripetalAccelerationConstraints,
                  startVelocity,
                  endVelocity,
                  maxVelocity,
                  maxAccel,
                  false);
    }

    public static Trajectory generateToBackLv1FromLoadingStation() {

        final double startVelocity = 0;
        final double endVelocity = 0;
        final double maxVelocity = Units.feetToMeters(11);
        final double maxAccel = Units.feetToMeters(5);

        ArrayList<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(Units.feetToMeters(1.431), Units.feetToMeters(2.154), Rotation2d.fromDegrees(180.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(18.769), Units.feetToMeters(4.954), Rotation2d.fromDegrees(180.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(23.064), Units.feetToMeters(3.077), Rotation2d.fromDegrees(-150.0)));

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

  /**
   * Contains paths for lv 1 and 2 front hatches
   */

  public static class RightRocketFront2 {

    public static Trajectory generateToFrontLv1() {

      final double startVelocity = 0;
      final double endVelocity = 0.5;
      final double maxVelocity = Units.feetToMeters(12);
      final double maxAccel = Units.feetToMeters(5);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(5.282), Units.feetToMeters(9.821), Rotation2d.fromDegrees(0)));
      waypoints.add(new Pose2d(Units.feetToMeters(14.447), Units.feetToMeters(3.577), Rotation2d.fromDegrees(-30.0)));

      ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
      centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(5)));

      return TrajectoryGenerator.generateTrajectory(
              waypoints,
              centripetalAccelerationConstraints,
              startVelocity,
              endVelocity,
              maxVelocity,
              maxAccel,
              false);
    }

    public static Trajectory generateBackUpFrontLv1() {

      final double startVelocity = 0;
      final double endVelocity = 0;
      final double maxVelocity = Units.feetToMeters(12);
      final double maxAccel = Units.feetToMeters(7);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(16.275), Units.feetToMeters(2.474), Rotation2d.fromDegrees(-30.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(12.688), Units.feetToMeters(6.057), Rotation2d.fromDegrees(-90.0)));
      //waypoints.add(new Pose2d(Units.feetToMeters(14.182), Units.feetToMeters(2.847), Rotation2d.fromDegrees(-31.0)));

      ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
      centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(6)));

      return TrajectoryGenerator.generateTrajectory(
              waypoints,
              centripetalAccelerationConstraints,
              startVelocity,
              endVelocity,
              maxVelocity,
              maxAccel,
              true);
    }

    public static Trajectory generateToLoadingStation() {

      final double startVelocity = 0;
      final double endVelocity = 0;
      final double maxVelocity = Units.feetToMeters(13);
      final double maxAccel = Units.feetToMeters(7);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(12.688), Units.feetToMeters(6.057), Rotation2d.fromDegrees(-90.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(9.169), Units.feetToMeters(1.791), Rotation2d.fromDegrees(-180.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(3.232), Units.feetToMeters(1.01), Rotation2d.fromDegrees(180.0)));

      ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
      centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(6)));

      return TrajectoryGenerator.generateTrajectory(
              waypoints,
              centripetalAccelerationConstraints,
              startVelocity,
              endVelocity,
              maxVelocity,
              maxAccel,
              false);
    }

    public static Trajectory  generateToFrontLv2FromLoading() {

      final double startVelocity = 0;
      final double endVelocity = 0;
      final double maxVelocity = Units.feetToMeters(13);
      final double maxAccel = Units.feetToMeters(7);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(1.475), Units.feetToMeters(2.154), Rotation2d.fromDegrees(-180.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(8.963), Units.feetToMeters(2.75), Rotation2d.fromDegrees(-180.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(13.302), Units.feetToMeters(5.372), Rotation2d.fromDegrees(-33.0)));

      ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
      centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(6)));

      return TrajectoryGenerator.generateTrajectory(
              waypoints,
              centripetalAccelerationConstraints,
              startVelocity,
              endVelocity,
              maxVelocity,
              maxAccel,
              true);
    }

    public static Trajectory generateLevel2Backup() {

      final double startVelocity = 0;
      final double endVelocity = 0;
      final double maxVelocity = Units.feetToMeters(2);
      final double maxAccel = Units.feetToMeters(1);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(16.6), Units.feetToMeters(2.204), Rotation2d.fromDegrees(-31.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(14.094), Units.feetToMeters(3.714), Rotation2d.fromDegrees(-32.0)));

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

  /**
   * Contains paths for 2 hatch cargo and rocket right side
   */

  public static class Right2HatchCargoRocket {

    public static Trajectory generateToCargo1() {

      final double startVelocity = 0;
      final double endVelocity = 0;
      final double maxVelocity = Units.feetToMeters(10);
      final double maxAccel = Units.feetToMeters(4);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(5.301), Units.feetToMeters(9.738), Rotation2d.fromDegrees(0.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(10.502), Units.feetToMeters(9.74), Rotation2d.fromDegrees(0.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(21.701), Units.feetToMeters(7.795), Rotation2d.fromDegrees(90.0)));

      ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
      centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(5)));

      return TrajectoryGenerator.generateTrajectory(
              waypoints,
              centripetalAccelerationConstraints,
              startVelocity,
              endVelocity,
              maxVelocity,
              maxAccel,
              false);
    }

    public static Trajectory generateCargo1BackUp() {

      final double startVelocity = 0;
      final double endVelocity = 0;
      final double maxVelocity = Units.feetToMeters(8);
      final double maxAccel = Units.feetToMeters(5);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(21.659), Units.feetToMeters(10.052), Rotation2d.fromDegrees(90.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(23.254), Units.feetToMeters(6.653), Rotation2d.fromDegrees(180.0)));

      ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
      centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(5)));

      return TrajectoryGenerator.generateTrajectory(
              waypoints,
              centripetalAccelerationConstraints,
              startVelocity,
              endVelocity,
              maxVelocity,
              maxAccel,
              true);
    }

    public static Trajectory generateCargo1ToLoadingStation() {

      final double startVelocity = 0;
      final double endVelocity = 0;
      final double maxVelocity = Units.feetToMeters(10);
      final double maxAccel = Units.feetToMeters(5);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(23.254), Units.feetToMeters(6.653), Rotation2d.fromDegrees(180.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(2.85), Units.feetToMeters(2.194), Rotation2d.fromDegrees(180.0)));

      ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
      centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(5)));

      return TrajectoryGenerator.generateTrajectory(
              waypoints,
              centripetalAccelerationConstraints,
              startVelocity,
              endVelocity,
              maxVelocity,
              maxAccel,
              false);
    }

    public static Trajectory generateToFrontLv1FromLoading() {

      final double startVelocity = 0;
      final double endVelocity = 0;
      final double maxVelocity = Units.feetToMeters(10);
      final double maxAccel = Units.feetToMeters(5);

      ArrayList<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(Units.feetToMeters(1.475), Units.feetToMeters(2.154), Rotation2d.fromDegrees(-180.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(8.963), Units.feetToMeters(2.107), Rotation2d.fromDegrees(-180.0)));
      waypoints.add(new Pose2d(Units.feetToMeters(14.357), Units.feetToMeters(4.765), Rotation2d.fromDegrees(-33.0)));

      ArrayList<TrajectoryConstraint> centripetalAccelerationConstraints = new ArrayList<>();
      centripetalAccelerationConstraints.add(new CentripetalAccelerationConstraint(Units.feetToMeters(5)));

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
}
