package frc.robot.config;

public class Target {

  private final int target;
  private final int upperBound;

  public Target(int target, int upperBound) {
    this.target = target;
    this.upperBound = upperBound;
  }

  public Target(int target) {
    this(target, 0);
  }

  public int getTarget() {
    return target;
  }

  public int getUpperBound() {
    return upperBound;
  }

  public boolean inRange(int angle) {
    return angle < upperBound;
  }

  public boolean isClose(int angle) {
    return Math.abs(angle - target) < 10;
  }

  @Override
  public String toString() {
    return "Target{" +
        "target=" + target +
        ", upperBound=" + upperBound +
        '}';
  }
}
