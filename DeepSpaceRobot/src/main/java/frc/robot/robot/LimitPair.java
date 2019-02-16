package frc.robot.robot;
public class LimitPair{
  private final int forwardsSoftLimitThreshold;
  private final int reverseSoftLimitThreshold;

  public LimitPair(int forwardsSoftLimitThreshold, int reverseSoftLimitThreshold) {
    this.forwardsSoftLimitThreshold = forwardsSoftLimitThreshold;
    this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
  }

  public int getForwardsSoftLimitThreshold() {
    return forwardsSoftLimitThreshold;
  }

  public int getReverseSoftLimitThreshold() {
    return reverseSoftLimitThreshold;
  }
}
