package frc.robot.command.teleop;

public class Sqrt implements Transform{

    @Override
    public double transform(double input) {
        System.out.println(input);
        double result = Math.signum(input) * Math.sqrt(Math.abs(input));
        System.out.println(result);
        return result;
    }

}
