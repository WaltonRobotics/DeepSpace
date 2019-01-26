package frc.robot.command.teleop.util;

import frc.robot.command.teleop.Transform;

public class NormalSpeed implements Transform {

  @Override
  public double transform(double input) {
    return input;
  }

}
