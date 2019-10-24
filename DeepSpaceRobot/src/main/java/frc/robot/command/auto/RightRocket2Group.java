package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths;
import lib.Utils.RamseteCommand;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.rightRocketToBack;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightRocket2Group extends CommandGroup {
    public RightRocket2Group() {
        addSequential(new RamseteCommand(Paths.generateToFrontRightRocket(),
                drivetrain::updateRobotPose,
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
        addSequential(new SetHatchIntake(false));
    }
}
