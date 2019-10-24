package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths;
import lib.Utils.RamseteCommand;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightRocketGroup extends CommandGroup {

    public RightRocketGroup() {
        addSequential(new RamseteCommand(Paths.generateToRightRocket1(),
                drivetrain::updateRobotPoseOffset,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));

        addSequential(new VisionAlign());
    }
}
