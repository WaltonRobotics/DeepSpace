/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 LUKELMAOXDRAWR. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.command.teleop;

import static frc.robot.OI.*;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Add your docs here.
 */
public class DriverTakeOver extends Command {

    double[] noActivity = {0, 0};
    double[] controllerAct = new double[2];
    double[] activity = new double[2];
    Log eventLog = new Log();
    boolean takeOver;

    //needs to be called somewhere
    public double[] DriverTakeOver() {
        while (isFinished() == false) {
            eventLog.leftJoystickActivity = leftJoystick.getY();
            eventLog.rightJoystickActivity = rightJoystick.getY();
            controllerAct = new double[]{eventLog.leftJoystickActivity, eventLog.rightJoystickActivity};
            return activity = controllerAct;
        }
        return new double[0];
    }

    public void isTakingOver() {
        takeOver = (activity == noActivity ? false: true);
    }

    public boolean getIsTakingOver() {
        return takeOver;
    }

	    @Override
	    protected boolean isFinished() {
		    return false;
	    }
    
    }
   
    