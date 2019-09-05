package lib.Utils;

public class VelocityPair {

    private double leftVelocity;
    private double rightVelocity;

    public VelocityPair(double leftVelocity, double rightVelocity) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
    }

    public VelocityPair getVelocityPair(double leftVelocity, double rightVelocity) {
      return new VelocityPair(leftVelocity, rightVelocity);
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }
}
