package frc.robot.command.teleop.util;


public class Sigmoid implements Transform {

  @Override
  public double transform(double input) {
    return 1.0 / (1.0 + StrictMath.pow(Math.E, -input));
  }
  // e = 2.7182818285
}