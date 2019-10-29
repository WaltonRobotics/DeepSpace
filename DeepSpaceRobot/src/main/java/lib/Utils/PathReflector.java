package lib.Utils;

import java.util.ArrayList;
import java.util.List;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.trajectory.Trajectory;
import lib.trajectory.Trajectory.State;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.Matrix;
import org.ejml.dense.row.CommonOps_DDRM;
import org.waltonrobotics.util.Polynomial;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.tan;
import static java.lang.StrictMath.sin;

/**
 * @author Russell Newton
 **/
public final class PathReflector {

  /**
   *
   * @param trajectory
   * @param point
   * @param angle radians
   * @return
   */
  public static Trajectory reflectOnLineThroughPoint(Trajectory trajectory, Pose2d point, double angle) {
//    trajectory.getStates().
    List<State> newStates = new ArrayList<>();
    trajectory.getStates().parallelStream().
        forEach(n -> newStates.add(reflectState(n, point, angle)));
    
    return new Trajectory(newStates);
  }

  private static State reflectState(State state, Pose2d point, double angle) {
    return new State(state.timeSeconds,
        state.velocityMetersPerSecond,
        state.accelerationMetersPerSecondSq,
        reflect(state.poseMeters, point, angle), state.curvatureRadPerMeter);
  }

  private static Pose2d reflect(Pose2d toReflect, Pose2d throughPoint, double angle) {
    double yIntercept = -throughPoint.getTranslation().getX() * tan(angle) +
        throughPoint.getTranslation().getY();
    double[][] shiftDown = {
        {toReflect.getTranslation().getX()},
        {toReflect.getTranslation().getY() - yIntercept},
        {1}
    };
    double[][] reflect = {
        {cos(2 * angle), sin(2 * angle), 0},
        {sin(2 * angle), -cos(2 * angle), 0},
        {1, 0, 0}
    };
    double[][] shiftBack = {
        {1, 0, 0},
        {0, 1, yIntercept},
        {0, 0, 1}
    };
    DMatrixRMaj transform = new DMatrixRMaj(shiftBack);
    CommonOps_DDRM.mult(transform, new DMatrixRMaj(reflect), transform);
    CommonOps_DDRM.mult(transform, new DMatrixRMaj(shiftDown), transform);

    return new Pose2d(transform.get(0, 0), transform.get(1, 0),
        new Rotation2d(2 * angle - toReflect.getRotation().getRadians()));
  }

}
