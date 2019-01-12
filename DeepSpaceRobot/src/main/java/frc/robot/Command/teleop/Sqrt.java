package frc.robot.command.teleop;
// sqrt
public class Sqrt implements Transform{

    @Override
    public double transform(double input) {
        return Math.sqrt(Math.abs(input));
    }

}
// sigmoid 
// divide by 2
// norm