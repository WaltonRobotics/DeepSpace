package frc.robot.command.teleop.responseFunction;

public class CubicResponse implements ResponseFunction {

    @Override
    public double getOutput(double input) {
        return Math.cbrt(input);
    }

}
