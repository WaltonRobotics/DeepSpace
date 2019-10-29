package frc.robot.command.teleop.util;

/**
 * @author Russell Newton
 **/
public class Squared implements Transform {

  @Override
  public double transform(double input) {
    return Math.copySign(input * input, input);
  }
}
