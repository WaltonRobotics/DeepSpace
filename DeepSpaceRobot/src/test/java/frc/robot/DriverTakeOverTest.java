/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 LUKELMAOXDRAWR. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.OI.leftJoystick;
import static frc.team2974.robot.OI.rightJoystick;


/**
 * Add your docs here.
 */
public class DriverTakeOverTest extends Command {

    public static double[] noActivity = {0, 0};
    public static double[] controllerAct = new double[2];
    public static double[] activity = new double[2];
    public static double tolerance = 1;
    Log eventLog = new Log();
    public static boolean takeOver;

    //needs to be called somewhere
    public double[] DriverTakeOver() {
        while (isFinished() == false) {
            eventLog.leftJoystickActivity = leftJoystick.getY();
            eventLog.rightJoystickActivity = rightJoystick.getY();
            controllerAct = new double[]{eventLog.leftJoystickActivity, eventLog.rightJoystickActivity};
            activity = controllerAct;
            return controllerAct;
        }
        return new double[0];
    }

    public static void isTakingOver() {
        takeOver = (!(activity[0] < noActivity[0] + tolerance && activity[1] < noActivity[1] + tolerance));
    }

    public static boolean getIsTakingOver() {
        return takeOver;
    }

	    @Override
	    protected boolean isFinished() {
		    return false;
	    }
    
    }
   
    