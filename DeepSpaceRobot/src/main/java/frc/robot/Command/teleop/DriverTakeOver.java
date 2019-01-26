package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.OI.leftJoystick;
import static frc.team2974.robot.OI.rightJoystick;

public class DriverTakeOver extends Command{

    public static double[] noActivity = {0, 0};
    public static double[] controllerAct = new double[2];
    public static double[] activity = new double[2];
    Log eventLog = new Log();
    public static boolean takeOver;

    //needs to be called somewhere
    public double[] DriverTakeOver() {
        while (!isFinished()) {
            eventLog.leftJoystickActivity = leftJoystick.getY();
            eventLog.rightJoystickActivity = rightJoystick.getY();
            controllerAct = new double[]{eventLog.leftJoystickActivity, eventLog.rightJoystickActivity};
            activity = controllerAct;
            return controllerAct;
        }
            return new double[0];
        }


        public static void isTakingOver() {

            takeOver = (!(activity[0] == noActivity[0] && activity[1] == noActivity[1]));

        }

        public static boolean getIsTakingOver() {
            return takeOver;
        }

        @Override
        protected boolean isFinished() {
            return false;
        }

    }



