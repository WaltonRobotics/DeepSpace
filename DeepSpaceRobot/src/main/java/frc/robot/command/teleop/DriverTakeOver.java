/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 LUKELMAOXDRAWR. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.command.teleop;

import static frc.robot.OI.*;



/**
 * Add your docs here.
 */
public class EventLogger {
    
    Log eventLog = new Log();

    public double[] eventActivity() {

        //need a while loop
        eventLog.leftJoystickActivity = leftJoystick.getY();
        eventLog.rightJoystickActivity = rightJoystick.getY();
        double[] activity = {eventLog.leftJoystickActivity, eventLog.rightJoystickActivity};
        return activity;
    
    }
}   



