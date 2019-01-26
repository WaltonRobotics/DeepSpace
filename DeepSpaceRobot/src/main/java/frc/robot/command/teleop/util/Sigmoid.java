package frc.robot.command.teleop.util;

import frc.robot.command.teleop.Transform;

public class Sigmoid implements Transform {

  @Override
  public double transform(double input) {
    return 1 / (1 + Math.pow(Math.E, -input));
  }
  // e = 2.7182818285
}