package frc.robot;

import frc.robot.command.teleop.HalfSpeed;
import frc.robot.command.teleop.Transform;
import org.junit.Test;

public class ControllerTestHalfSpeed {
    @Test
    public void halfSpeed(){
        Transform half = new HalfSpeed();
        System.out.println(half.transform(1));
    }
}
