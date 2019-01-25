package frc.robot.Command.teleop.util;


public class Sigmoid implements frc.robot.command.teleop.util.Transform {

  @Override
  public double transform(double input) {
    return 1 / (1 + Math.pow(Math.E, -input));
  }
  // e = 2.7182818285
}