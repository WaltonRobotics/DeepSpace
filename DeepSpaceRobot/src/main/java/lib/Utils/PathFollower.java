package lib.Utils;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import lib.Controller.RamseteController;
import lib.Controller.RamseteController.Outputs;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;

public class PathFollower {

  private int currentSegIndex;
  private Segment currentSeg;
  private Trajectory trajectory;

  private double drive_radius;

  private RamseteController ramseteController;

  public PathFollower(Trajectory traj, double kBeta, double kZeta, double driveRadius) {
      ramseteController = new RamseteController(kBeta, kZeta);

      trajectory = traj;
      currentSegIndex = 0;
      currentSeg = trajectory.segments[currentSegIndex];
      drive_radius = driveRadius;
  }

  /**
   * @param pose : The current pose of the robot
   * @return A velocity pair of the left and right wheel speeds
   */
  public VelocityPair getRobotVelocity(Pose2d pose) {

    // Update Segment Safely
    if (isFinished())
      return new VelocityPair(0, 0);

    // Calculate X and Y error
    double xError = currentSeg.x - pose.getTranslation().getX();
    double yError = currentSeg.y - pose.getTranslation().getY();

    // Calculate Linear Velocity of Segment

    double sv = currentSeg.velocity;

    // Calculate Angular Velocity of Segment

    double sw = isFinished() ? 0 : (trajectory.segments[currentSegIndex + 1].heading - currentSeg.heading)/currentSeg.dt;

    // Calculate linear and angular velocity based on errors

    Outputs ramseteOutputs = ramseteController
        .calculate(new Pose2d(currentSeg.x, currentSeg.y, new Rotation2d(currentSeg.heading)), sv, sw, pose);

    ChassisSpeeds speeds = new ChassisSpeeds(ramseteOutputs.linearVelocity,0,ramseteOutputs.angularVelocity);

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(drive_radius);

    currentSegIndex++;

    return new VelocityPair(kinematics.toWheelSpeeds(speeds).left, kinematics.toWheelSpeeds(speeds).right);
  }

  private boolean isFinished() {
    return currentSegIndex >= trajectory.length();
  }
}
