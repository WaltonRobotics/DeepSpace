package frc.robot.command.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Utils.MotionPair;
import lib.trajectory.Trajectory;

import static frc.robot.Config.RamseteControllerConstants.DRIVE_RADIUS;
import static frc.robot.Config.RamseteControllerConstants.K_BETA;
import static frc.robot.Config.RamseteControllerConstants.K_ZETA;
import static frc.robot.Config.RamseteControllerConstants.TOLERANCE_POSE;
import static frc.robot.Config.SmartMotionConstants.KT;
import static frc.robot.Robot.drivetrain;

public class FollowTrajectory extends Command {

  private DifferentialDriveKinematics kinematics;

  private Pose2d startingPose;
  private Trajectory trajectory;

  private double currentTime;
  private double dt;

  private RamseteController ramseteController;

  private Timer timer;

  private boolean backwards;

  private double leftVelocity;
  private double rightVelocity;
  private double leftAcceleration;
  private double rightAcceleration;

  private double previousLeftVelocity;
  private double previousRightVelocity;

  public FollowTrajectory(Trajectory trajectory, boolean isBackwards) {
    requires(drivetrain);
    this.startingPose = trajectory.getStates().get(0).poseMeters;
    this.trajectory = trajectory;
    this.kinematics = new DifferentialDriveKinematics(DRIVE_RADIUS);
    this.ramseteController = new RamseteController(K_BETA, K_ZETA);
    this.currentTime = 0;
    this.dt = KT;
    this.leftVelocity = 0;
    this.rightVelocity = 0;
    this.leftAcceleration = 0;
    this.rightAcceleration = 0;
    this.previousLeftVelocity = 0;
    this.previousRightVelocity = 0;
    this.backwards = isBackwards;
    this.timer = new Timer();
  }

  @Override
  protected void initialize() {
    drivetrain.getDriveOdometry().resetPosition(startingPose);
    ramseteController.setTolerance(TOLERANCE_POSE);
    timer.start();
  }

  @Override
  protected void execute() {
    Pose2d currentPose = drivetrain.updateRobotPose();
    MotionPair drivetrainMotions = getRobotMotions(currentPose);

    if (backwards) {
      drivetrain.setVoltages(-drivetrainMotions.getRightVelocity(), -drivetrainMotions.getRightAcceleration(), -drivetrainMotions.getLeftVelocity(), -drivetrainMotions.getLeftAcceleration());
    } else {
      drivetrain.setVoltages(drivetrainMotions.getLeftVelocity(), drivetrainMotions.getLeftAcceleration(), drivetrainMotions.getRightVelocity(), drivetrainMotions.getRightAcceleration());
    }
  }

  @Override
  protected boolean isFinished() {
    return ramseteController.atReference();
  }

  @Override
  protected void end() {
    timer.stop();
  }

  /**
   * @param currentPose : The current pose of the robot
   * @return A chassis speed of the robot
   */

  private MotionPair getRobotMotions(Pose2d currentPose) {

    currentTime = timer.get();

    Trajectory.State desiredState = trajectory.sample(currentTime);
    ChassisSpeeds ramseteOutputs = ramseteController.calculate(currentPose, desiredState);

    leftVelocity = kinematics.toWheelSpeeds(ramseteOutputs).leftMetersPerSecond;
    rightVelocity = kinematics.toWheelSpeeds(ramseteOutputs).rightMetersPerSecond;

    leftAcceleration = (leftVelocity - previousLeftVelocity) / dt;
    rightAcceleration = (rightVelocity - previousRightVelocity) / dt;

    previousLeftVelocity = leftVelocity;
    previousRightVelocity = rightVelocity;

    return new MotionPair(leftVelocity, leftAcceleration, rightVelocity, rightAcceleration);
  }

  public double getLeftVelocity() {
    return leftVelocity;
  }

  public double getRightVelocity() {
    return rightVelocity;
  }

  public double getLeftAcceleration() {
    return leftAcceleration;
  }

  public double getRightAcceleration() {
    return rightAcceleration;
  }
}

