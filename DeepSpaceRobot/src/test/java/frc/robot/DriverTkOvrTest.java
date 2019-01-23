package frc.robot;

import frc.robot.command.teleop.DriverTakeOver;
import org.junit.Test;

public class DriverTkOvrTest {
    @Test
    public void TakeOverTest() {
        DriverTakeOver tkOver = new DriverTakeOver();
        tkOver.DriverTakeOver();
        tkOver.isTakingOver();
        System.out.println(tkOver.getIsTakingOver());
    }

    }

