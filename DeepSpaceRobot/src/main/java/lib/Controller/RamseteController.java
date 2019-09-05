package lib.Controller;

import lib.Geometry.Pose2d;

/**
 * Ramsete is a nonlinear time-varying feedback controller for unicycle models
 * that drives the model to a desired pose along a two-dimensional trajectory.
 * Why would we need a nonlinear control law in addition to the linear ones we
 * have used so far like PID? If we use the original approach with PID
 * controllers for left and right position and velocity states, the controllers
 * only deal with the local pose. If the robot deviates from the path, there is
 * no way for the controllers to correct and the robot may not reach the desired
 * global pose. This is due to multiple endpoints existing for the robot which
 * have the same encoder path arc lengths.
 *
 * <p>Instead of using wheel path arc lengths (which are in the robot’s local
 * coordinate frame), nonlinear controllers like pure pursuit and Ramsete use
 * global pose. The controller uses this extra information to guide a linear
 * reference tracker like the PID controllers back in by adjusting the
 * references of the PID controllers.
 *
 * <p>The paper "Control of Wheeled Mobile Robots: An Experimental Overview"
 * describes a nonlinear controller for a wheeled vehicle with unicycle-like
 * kinematics; a global pose consisting of x, y, and θ; and a desired pose
 * consisting of x_d, y_d, and θ_d. We call it Ramsete because that’s the
 * acronym for the title of the book it came from in Italian (“Robotica
 * Articolata e Mobile per i SErvizi e le TEcnologie”).
 *
 * <p>See <a href="https://file.tavsys.net/control/state-space-guide.pdf">
 * Controls Engineering in the FIRST Robotics Competition</a>
 * section on Ramsete unicycle controller for a derivation and analysis.
 */
public class RamseteController {
  @SuppressWarnings("MemberName")
  private final double m_b;
  @SuppressWarnings("MemberName")
  private final double m_zeta;

  @SuppressWarnings("MemberName")
  public class Outputs {
    public double linearVelocity;
    public double angularVelocity;

    public Outputs(double linearVelocity, double angularVelocity) {
      this.linearVelocity = linearVelocity;
      this.angularVelocity = angularVelocity;
    }
  }

  /**
   * Construct a Ramsete unicycle controller.
   *
   * @param b    Tuning parameter (b &gt; 0) for which larger values make convergence more
   *             aggressive like a proportional term.
   * @param zeta Tuning parameter (0 &lt; zeta &lt; 1) for which larger values provide more damping
   *             in response.
   */
  @SuppressWarnings("ParameterName")
  public RamseteController(double b, double zeta) {
    m_b = b;
    m_zeta = zeta;
  }

  /**
   * Returns the next output of the Ramsete controller.
   *
   * <p>The desired pose, linear velocity, and angular velocity should come from a drivetrain
   * trajectory.
   *
   * @param desiredPose            The desired pose.
   * @param desiredLinearVelocity  The desired linear velocity.
   * @param desiredAngularVelocity The desired angular velocity.
   * @param currentPose            The current pose.
   */
  @SuppressWarnings("LocalVariableName")
  public Outputs calculate(Pose2d desiredPose,
      double desiredLinearVelocity,
      double desiredAngularVelocity,
      Pose2d currentPose) {
      var error = desiredPose.relativeTo(currentPose);

    // Aliases for equation readability
    double vDesired = desiredLinearVelocity;
    double omegaDesired = desiredAngularVelocity;
    double eX = error.getTranslation().getX();
    double eY = error.getTranslation().getY();
    double eTheta = error.getRotation().getRadians();

    double k = 2.0 * m_zeta
        * Math.sqrt(Math.pow(omegaDesired, 2) + m_b * Math.pow(vDesired, 2));

      return new Outputs(vDesired * Math.cos(eTheta) + k * eX,
        omegaDesired + k * eTheta + m_b * vDesired * sinc(eTheta) * eY);
  }

  /**
   * Returns sin(x) / x.
   *
   * @param x Value of which to take sinc(x).
   */
  @SuppressWarnings("ParameterName")
  private static double sinc(double x) {
    if (Math.abs(x) < 1e-9) {
      return 1.0 - 1.0 / 6.0 * x * x;
    } else {
      return Math.sin(x) / x;
    }
  }
}
