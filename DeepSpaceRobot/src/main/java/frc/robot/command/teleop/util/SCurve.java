package frc.robot.command.teleop.util;

/**
 * @author Russell Newton
 **/
public class SCurve implements Transform{
  @Override
  public double transform(double input) {
    return 3 * (1 - input) * Math.pow(input, 2) + Math.pow(input, 3);
  }
}
