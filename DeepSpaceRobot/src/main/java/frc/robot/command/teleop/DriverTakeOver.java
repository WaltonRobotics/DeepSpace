/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 LUKELMAOXDRAWR. All Rights Reserved.                             */
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
    double[] activity = new double[2];
    Log eventLog = new Log();
    public DriverTakeOver() {
        while (isFinished() == false) {
            eventLog.leftJoystickActivity = leftJoystick.getY();
            eventLog.rightJoystickActivity = rightJoystick.getY();
            double[] controllerAct = {eventLog.leftJoystickActivity, eventLog.rightJoystickActivity};
            activity = controllerAct;
        }   
    }

    @Override
	protected boolean isFinished() {
		return false;
    }
    
    boolean takeOver;

    public void isTakingOver() {
        if(isFinished() == true) {
            takeOver = false;
        }
            takeOver = true;
    }
}    
    