package lib.Utils;

public class MotionPair {
    private double leftVelocity;
    private double rightVelocity;

    private double leftAcceleration;
    private double rightAcceleration;

    public MotionPair(double leftVelocity, double leftAcceleration, double rightVelocity, double rightAcceleration) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.leftAcceleration = leftAcceleration;
        this.rightAcceleration = rightAcceleration;
    }

    public MotionPair getMotionPair(double leftVelocity, double leftAcceleration, double rightVelocity, double rightAcceleration) {
      return new MotionPair(leftVelocity, leftAcceleration, rightVelocity, rightAcceleration);
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getLeftAcceleration() {
        return leftAcceleration;
    }

    public double getRightAcceleration() {
        return rightAcceleration;
    }
}
