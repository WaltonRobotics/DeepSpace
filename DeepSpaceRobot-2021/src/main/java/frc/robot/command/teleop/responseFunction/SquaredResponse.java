package frc.robot.command.teleop.responseFunction;

public class SquaredResponse implements ResponseFunction {

    @Override
    public double getOutput(double input) {
        return Math.copySign(input * input, input);
    }

}
