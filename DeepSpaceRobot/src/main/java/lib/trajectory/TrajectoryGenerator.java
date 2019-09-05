package lib.trajectory;

import java.util.ArrayList;
import java.util.List;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Geometry.Transform2d;
import lib.Geometry.Translation2d;
import lib.Spline.PoseWithCurvature;
import lib.Spline.Spline;
import lib.Spline.SplineHelper;
import lib.trajectory.constraint.TrajectoryConstraint;

public final class TrajectoryGenerator {
  /**
   * Private constructor because this is a utility class.
   */
  private TrajectoryGenerator() {
  }

  /**
   * Generates a trajectory with the given waypoints and constraints.
   *
   * @param waypoints                        A vector of points that the trajectory must go through.
   * @param constraints                      A vector of various velocity and acceleration
   *                                         constraints.
   * @param startVelocityMetersPerSecond     The start velocity for the trajectory.
   * @param endVelocityMetersPerSecond       The end velocity for the trajectory.
   * @param maxVelocityMetersPerSecond       The max velocity for the trajectory.
   * @param maxAccelerationMetersPerSecondSq The max acceleration for the trajectory.
   * @param reversed                         Whether the robot should move backwards. Note that the
   *                                         robot will still move from a -> b -> ... -> z
   *                                         as defined in the waypoints.
   * @return The trajectory.
   */

  public static Trajectory generateTrajectory(
      List<Pose2d> waypoints,
      List<TrajectoryConstraint> constraints,
      double startVelocityMetersPerSecond,
      double endVelocityMetersPerSecond,
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      boolean reversed
  ) {
    final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-180.0));

    // Make theta normal for trajectory generation if path is reversed.
    final var newWaypoints = new ArrayList<Pose2d>(waypoints.size());
    for (final var point : waypoints) {
      newWaypoints.add(reversed ? point.plus(flip) : point);
    }

    var points = splinePointsFromSplines(SplineHelper.getQuinticSplinesFromWaypoints(
        newWaypoints.toArray(new Pose2d[0])
    ));

    // After trajectory generation, flip theta back so it's relative to the
    // field. Also fix curvature.
    if (reversed) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(flip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    return timeParameterizeTrajectory(points, constraints,
        startVelocityMetersPerSecond, endVelocityMetersPerSecond, maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSq, reversed);
  }

  /**
   * Generates a trajectory with the given waypoints and constraints.
   *
   * @param start                            The starting pose for the trajectory.
   * @param waypoints                        The interior waypoints for the trajectory. The headings
   *                                         will be determined automatically to ensure continuous
   *                                         curvature.
   * @param end                              The ending pose for the trajectory.
   * @param constraints                      A vector of various velocity and acceleration
   *                                         constraints.
   * @param startVelocityMetersPerSecond     The start velocity for the trajectory.
   * @param endVelocityMetersPerSecond       The end velocity for the trajectory.
   * @param maxVelocityMetersPerSecond       The max velocity for the trajectory.
   * @param maxAccelerationMetersPerSecondSq The max acceleration for the trajectory.
   * @param reversed                         Whether the robot should move backwards. Note that the
   *                                         robot will still move from a -> b -> ... -> z as
   *                                         defined in the waypoints.
   * @return The trajectory.
   */
  public static Trajectory generateTrajectory(
      Pose2d start,
      List<Translation2d> waypoints,
      Pose2d end,
      List<TrajectoryConstraint> constraints,
      double startVelocityMetersPerSecond,
      double endVelocityMetersPerSecond,
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      boolean reversed
  ) {
    final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-180.0));

    final var newStart = reversed ? start.plus(flip) : start;
    final var newEnd = reversed ? end.plus(flip) : end;

    var points = splinePointsFromSplines(SplineHelper.getCubicSplinesFromWaypoints(
        newStart, waypoints.toArray(new Translation2d[0]), newEnd
    ));

    // After trajectory generation, flip theta back so it's relative to the
    // field. Also fix curvature.
    if (reversed) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(flip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    return timeParameterizeTrajectory(points, constraints,
        startVelocityMetersPerSecond, endVelocityMetersPerSecond, maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSq, reversed);
  }

  private static List<PoseWithCurvature> splinePointsFromSplines(
      Spline[] splines) {
    // Create the vector of spline points.
    var splinePoints = new ArrayList<PoseWithCurvature>();

    // Add the first point to the vector.
    splinePoints.add(splines[0].getPoint(0.0));

    // Iterate through the vector and parameterize each spline, adding the
    // parameterized points to the final vector.
    for (final var spline : splines) {
      var points = spline.parameterize();

      // Append the array of poses to the vector. We are removing the first
      // point because it's a duplicate of the last point from the previous
      // spline.
      splinePoints.addAll(points.subList(1, points.size()));
    }
    return splinePoints;
  }

  @SuppressWarnings({"PMD.ExcessiveMethodLength", "PMD.CyclomaticComplexity",
      "PMD.NPathComplexity", "PMD.AvoidInstantiatingObjectsInLoops",
      "PMD.AvoidThrowingRawExceptionTypes"})
  private static Trajectory timeParameterizeTrajectory(
      List<PoseWithCurvature> points,
      List<TrajectoryConstraint> constraints,
      double startVelocityMetersPerSecond,
      double endVelocityMetersPerSecond,
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      boolean reversed
  ) {
    var constrainedStates = new ArrayList<ConstrainedState>(points.size());
    var predecessor = new ConstrainedState(points.get(0), 0, startVelocityMetersPerSecond,
        -maxAccelerationMetersPerSecondSq, maxAccelerationMetersPerSecondSq);

    // Forward pass
    for (int i = 0; i < points.size(); i++) {
      constrainedStates.add(new ConstrainedState());
      var constrainedState = constrainedStates.get(i);
      constrainedState.pose = points.get(i);

      // Begin constraining based on predecessor.
      double ds = constrainedState.pose.poseMeters.getTranslation().getDistance(
          predecessor.pose.poseMeters.getTranslation());
      constrainedState.distanceMeters = predecessor.distanceMeters + ds;

      // We may need to iterate to find the maximum end velocity and common
      // acceleration, since acceleration limits may be a function of velocity.
      while (true) {
        // Enforce global max velocity and max reachable velocity by global
        // acceleration limit. vf = std::sqrt(vi^2 + 2*a*d).
        constrainedState.maxVelocityMetersPerSecond = Math.min(
            maxVelocityMetersPerSecond,
            Math.sqrt(predecessor.maxVelocityMetersPerSecond
                * predecessor.maxVelocityMetersPerSecond
                + predecessor.maxAccelerationMetersPerSecondSq * ds * 2.0)
        );

        constrainedState.minAccelerationMetersPerSecondSq = -maxAccelerationMetersPerSecondSq;
        constrainedState.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;

        // At this point, the constrained state is fully constructed apart from
        // all the custom-defined user constraints.
        for (final var constraint : constraints) {
          constrainedState.maxVelocityMetersPerSecond = Math.min(
              constrainedState.maxVelocityMetersPerSecond,
              constraint.getMaxVelocityMetersPerSecond(
                  constrainedState.pose.poseMeters, constrainedState.pose.curvatureRadPerMeter)
          );
        }

        // Now enforce all acceleration limits.
        enforceAccelerationLimits(reversed, constraints, constrainedState);

        if (ds < 1E-6) {
          break;
        }

        // If the actual acceleration for this state is higher than the max
        // acceleration that we applied, then we need to reduce the max
        // acceleration of the predecessor and try again.
        double actualAcceleration = (constrainedState.maxVelocityMetersPerSecond
            * constrainedState.maxVelocityMetersPerSecond
            - predecessor.maxVelocityMetersPerSecond * predecessor.maxVelocityMetersPerSecond)
            / (ds * 2.0);

        // If we violate the max acceleration constraint, let's modify the
        // predecessor.
        if (constrainedState.maxAccelerationMetersPerSecondSq < actualAcceleration - 1E-6) {
          predecessor.maxAccelerationMetersPerSecondSq
              = constrainedState.maxAccelerationMetersPerSecondSq;
        } else {
          // Constrain the predecessor's max acceleration to the current
          // acceleration.
          if (actualAcceleration > predecessor.minAccelerationMetersPerSecondSq) {
            predecessor.maxAccelerationMetersPerSecondSq = actualAcceleration;
          }
          // If the actual acceleration is less than the predecessor's min
          // acceleration, it will be repaired in the backward pass.
          break;
        }
      }
      predecessor = constrainedState;
    }

    var successor = new ConstrainedState(points.get(points.size() - 1),
        constrainedStates.get(constrainedStates.size() - 1).distanceMeters,
        endVelocityMetersPerSecond,
        -maxAccelerationMetersPerSecondSq, maxAccelerationMetersPerSecondSq);

    // Backward pass
    for (int i = points.size() - 1; i >= 0; i--) {
      var constrainedState = constrainedStates.get(i);
      double ds = constrainedState.distanceMeters - successor.distanceMeters; // negative

      while (true) {
        // Enforce max velocity limit (reverse)
        // vf = std::sqrt(vi^2 + 2*a*d), where vi = successor.
        double newMaxVelocity = Math.sqrt(
            successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond
                + successor.minAccelerationMetersPerSecondSq * ds * 2.0
        );

        // No more limits to impose! This state can be finalized.
        if (newMaxVelocity >= constrainedState.maxVelocityMetersPerSecond) {
          break;
        }

        constrainedState.maxVelocityMetersPerSecond = newMaxVelocity;

        // Check all acceleration constraints with the new max velocity.
        enforceAccelerationLimits(reversed, constraints, constrainedState);

        if (ds > -1E-6) {
          break;
        }

        // If the actual acceleration for this state is lower than the min
        // acceleration, then we need to lower the min acceleration of the
        // successor and try again.
        double actualAcceleration = (constrainedState.maxVelocityMetersPerSecond
            * constrainedState.maxVelocityMetersPerSecond
            - successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond)
            / (ds * 2.0);

        if (constrainedState.minAccelerationMetersPerSecondSq > actualAcceleration + 1E-6) {
          successor.minAccelerationMetersPerSecondSq
              = constrainedState.minAccelerationMetersPerSecondSq;
        } else {
          successor.minAccelerationMetersPerSecondSq = actualAcceleration;
          break;
        }
      }
      successor = constrainedState;
    }

    // Now we can integrate the constrained states forward in time to obtain our
    // trajectory states.
    var states = new ArrayList<Trajectory.State>(points.size());
    double timeSeconds = 0.0;
    double distanceMeters = 0.0;
    double velocityMetersPerSecond = 0.0;

    for (int i = 0; i < constrainedStates.size(); i++) {
      final var state = constrainedStates.get(i);

      // Calculate the change in position between the current state and the previous
      // state.
      double ds = state.distanceMeters - distanceMeters;

      // Calculate the acceleration between the current state and the previous
      // state.
      double accel = (state.maxVelocityMetersPerSecond * state.maxVelocityMetersPerSecond
          - velocityMetersPerSecond * velocityMetersPerSecond) / (ds * 2);

      // Calculate dt
      double dt = 0.0;
      if (i > 0) {
        states.get(i - 1).accelerationMetersPerSecondSq = reversed ? -accel : accel;
        if (Math.abs(accel) > 1E-6) {
          // v_f = v_0 + a * t
          dt = (state.maxVelocityMetersPerSecond - velocityMetersPerSecond) / accel;
        } else if (Math.abs(velocityMetersPerSecond) > 1E-6) {
          // delta_x = v * t
          dt = ds / velocityMetersPerSecond;
        } else {
          throw new RuntimeException("Something went wrong");
        }
      }

      velocityMetersPerSecond = state.maxVelocityMetersPerSecond;
      distanceMeters = state.distanceMeters;

      timeSeconds += dt;

      states.add(new Trajectory.State(
          timeSeconds,
          reversed ? -velocityMetersPerSecond : velocityMetersPerSecond,
          reversed ? -accel : accel,
          state.pose.poseMeters, state.pose.curvatureRadPerMeter
      ));
    }

    return new Trajectory(states);
  }

  private static void enforceAccelerationLimits(boolean reverse,
      List<TrajectoryConstraint> constraints,
      ConstrainedState state) {

    for (final var constraint : constraints) {
      double factor = reverse ? -1.0 : 1.0;
      final var minMaxAccel = constraint.getMinMaxAccelerationMetersPerSecondSq(
          state.pose.poseMeters, state.pose.curvatureRadPerMeter,
          state.maxVelocityMetersPerSecond * factor);

      state.minAccelerationMetersPerSecondSq = Math.max(state.minAccelerationMetersPerSecondSq,
          reverse ? -minMaxAccel.maxAccelerationMetersPerSecondSq
              : minMaxAccel.minAccelerationMetersPerSecondSq);

      state.maxAccelerationMetersPerSecondSq = Math.min(state.maxAccelerationMetersPerSecondSq,
          reverse ? -minMaxAccel.minAccelerationMetersPerSecondSq
              : minMaxAccel.maxAccelerationMetersPerSecondSq);
    }

  }

  @SuppressWarnings("MemberName")
  private static class ConstrainedState {
    PoseWithCurvature pose;
    double distanceMeters;
    double maxVelocityMetersPerSecond;
    double minAccelerationMetersPerSecondSq;
    double maxAccelerationMetersPerSecondSq;

    ConstrainedState(PoseWithCurvature pose, double distanceMeters,
        double maxVelocityMetersPerSecond,
        double minAccelerationMetersPerSecondSq,
        double maxAccelerationMetersPerSecondSq) {
      this.pose = pose;
      this.distanceMeters = distanceMeters;
      this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
      this.minAccelerationMetersPerSecondSq = minAccelerationMetersPerSecondSq;
      this.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
    }

    ConstrainedState() {
      pose = new PoseWithCurvature();
    }
  }
}