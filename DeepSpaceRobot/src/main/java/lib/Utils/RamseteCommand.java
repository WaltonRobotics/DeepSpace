package lib.Utils;


import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveWheelSpeeds;
import lib.trajectory.Trajectory;

import static frc.robot.Robot.drivetrain;
import static lib.Utils.ErrorMessages.requireNonNullParam;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID and feedforward functionality, returning only the raw wheel speeds from the RAMSETE
 * controller.
 */
public class RamseteCommand extends Command {
  private final Timer m_timer = new Timer();
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final RamseteController m_follower;
  private final double m_ks;
  private final double m_kv;
  private final double m_ka;
  private final DifferentialDriveKinematics m_kinematics;
  private final DoubleSupplier m_leftSpeed;
  private final DoubleSupplier m_rightSpeed;
  private final PIDController m_leftController;
  private final PIDController m_rightController;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * PID control and feedforward are handled internally, and outputs are scaled -1 to 1 for easy
   * consumption by speed controllers.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this
   * is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory                     The trajectory to follow.
   * @param pose                           A function that supplies the robot pose - use one of
   *                                       the odometry classes to provide this.
   * @param follower                       The RAMSETE follower used to follow the trajectory.
   * @param ksVolts                        Constant feedforward term for the robot drive.
   * @param kvVoltSecondsPerMeter          Velocity-proportional feedforward term for the robot
   *                                       drive.
   * @param kaVoltSecondsSquaredPerMeter   Acceleration-proportional feedforward term for the robot
   *                                       drive.
   * @param kinematics                     The kinematics for the robot drivetrain.
   * @param leftWheelSpeedMetersPerSecond  A function that supplies the speed of the left side of
   *                                       the robot drive.
   * @param rightWheelSpeedMetersPerSecond A function that supplies the speed of the right side of
   *                                       the robot drive.
   * @param leftController                 The PIDController for the left side of the robot drive.
   * @param rightController                The PIDController for the right side of the robot drive.
   */
  @SuppressWarnings("PMD.ExcessiveParameterList")
  public RamseteCommand(Trajectory trajectory,
                        Supplier<Pose2d> pose,
                        RamseteController follower,
                        double ksVolts,
                        double kvVoltSecondsPerMeter,
                        double kaVoltSecondsSquaredPerMeter,
                        DifferentialDriveKinematics kinematics,
                        DoubleSupplier leftWheelSpeedMetersPerSecond,
                        DoubleSupplier rightWheelSpeedMetersPerSecond,
                        PIDController leftController,
                        PIDController rightController) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    m_pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    m_follower = requireNonNullParam(follower, "follower", "RamseteCommand");
    m_ks = ksVolts;
    m_kv = kvVoltSecondsPerMeter;
    m_ka = kaVoltSecondsSquaredPerMeter;
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
    m_leftSpeed = requireNonNullParam(leftWheelSpeedMetersPerSecond,
        "leftWheelSpeedMetersPerSecond",
        "RamseteCommand");
    m_rightSpeed = requireNonNullParam(rightWheelSpeedMetersPerSecond,
        "rightWheelSpeedMetersPerSecond",
        "RamseteCommand");
    m_leftController = requireNonNullParam(leftController, "leftController", "RamseteCommand");
    m_rightController = requireNonNullParam(rightController, "rightController", "RamseteCommand");

  }

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
   * from the RAMSETE controller, and will need to be converted into a usable form by the user.
   *
   * @param trajectory            The trajectory to follow.
   * @param pose                  A function that supplies the robot pose - use one of
   *                              the odometry classes to provide this.
   * @param follower              The RAMSETE follower used to follow the trajectory.
   * @param kinematics            The kinematics for the robot drivetrain.
   * @param outputMetersPerSecond A function that consumes the computed left and right
   *                              wheel speeds.
   */
  public RamseteCommand(Trajectory trajectory,
                        Supplier<Pose2d> pose,
                        RamseteController follower,
                        DifferentialDriveKinematics kinematics,
                        BiConsumer<Double, Double> outputMetersPerSecond) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    m_pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    m_follower = requireNonNullParam(follower, "follower", "RamseteCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");

    m_ks = 0;
    m_kv = 0;
    m_ka = 0;
    m_leftSpeed = null;
    m_rightSpeed = null;
    m_leftController = null;
    m_rightController = null;

  }

  @Override
  public void initialize() {
    m_prevTime = 0;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter
                * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (m_leftController != null) {
      double leftFeedforward =
          m_ks * Math.signum(leftSpeedSetpoint)
              + m_kv * leftSpeedSetpoint
              + m_ka * (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt;

      double rightFeedforward =
          m_ks * Math.signum(rightSpeedSetpoint)
              + m_kv * rightSpeedSetpoint
              + m_ka * (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt;

      leftOutput = leftFeedforward
          + m_leftController.calculate(leftSpeedSetpoint,
          m_leftSpeed.getAsDouble());

      rightOutput = rightFeedforward
          + m_rightController.calculate(rightSpeedSetpoint,
          m_rightSpeed.getAsDouble());
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    drivetrain.setVoltages(leftOutput, rightOutput);

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;

  }

  public void end(boolean interrupted) {
    m_timer.stop();
  }

  public boolean isFinished() {
    return m_timer.get() - m_trajectory.getTotalTimeSeconds() >= 0;
  }
}