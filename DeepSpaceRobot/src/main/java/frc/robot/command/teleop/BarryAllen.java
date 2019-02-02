/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command.teleop;

/**
 * Add your docs here.
 */
public class BarryAllen implements Transform{

    @Override
    public double transform(double input) {
        return Math.signum(input) * Math.pow(Math.abs(input), 2);
    }
}
