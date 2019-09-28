package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Utils.MotionPair;
import lib.trajectory.Trajectory;

import static frc.robot.Config.RamseteControllerConstants.K_BETA;
import static frc.robot.Config.RamseteControllerConstants.K_ZETA;
import static frc.robot.Robot.drivetrain;

public class FollowTrajectory extends Command {

  private DifferentialDriveKinematics kinematics;

  private Pose2d startingPose;
  private Trajectory trajectory;

  private double currentTime;
  private double dt;

  private RamseteController ramseteController;

  public FollowTrajectory(Pose2d startingPose, Trajectory trajectory, double driveRadius) {
    requires(drivetrain);
    this.startingPose = startingPose;
    this.trajectory = trajectory;
    this.kinematics = new DifferentialDriveKinematics(driveRadius);
    this.ramseteController = new RamseteController(K_BETA, K_ZETA);
    this.currentTime = 0;
    this.dt = 0.04;
  }

  @Override
  protected void initialize() {
    drivetrain.getDriveOdometry().resetPosition(startingPose);
    ramseteController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(30)));
  }


  @Override
  protected void execute() {
    Pose2d currentPose = drivetrain.updateRobotPose();
    MotionPair drivetrainMotions = getRobotVelocity(currentPose);
    drivetrain.setVoltages(drivetrainMotions.getLeftVelocity(), drivetrainMotions.getLeftAcceleration(), drivetrainMotions.getRightVelocity(), drivetrainMotions.getRightAcceleration());
  }

  @Override
  protected boolean isFinished() {
    return ramseteController.atReference();
  }

  /**
   * @param pose : The current pose of the robot
   * @return A chassis speed of the robot
   */

  private MotionPair getRobotVelocity(Pose2d pose) {

    Trajectory.State desiredState = trajectory.sample(currentTime);
    ChassisSpeeds ramseteOutputs = ramseteController.calculate(pose, desiredState);

    currentTime += dt;

    return new MotionPair(kinematics.toWheelSpeeds(ramseteOutputs).leftMetersPerSecond, 0, kinematics.toWheelSpeeds(ramseteOutputs).rightMetersPerSecond, 0);
  }

}
