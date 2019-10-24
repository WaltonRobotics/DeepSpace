package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import lib.Utils.RamseteCommand;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.*;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightRocketGroup extends CommandGroup {

    public RightRocketGroup() {
        addSequential(new RamseteCommand(rightRocketToBack,
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
        addSequential(new RamseteCommand(rightRocketBackUpBack,
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
//        addSequential(new RamseteCommand(rightRocketFromBackToLoading,
//                drivetrain::updateRobotPose,
//                drivetrain.ramseteController,
//                KS,
//                KV,
//                KA,
//                drivetrain.differentialDriveKinematics,
//                encoderLeft::getRate,
//                encoderRight::getRate,
//                drivetrain.m_leftPIDController,
//                drivetrain.m_rightPIDController));
//        addSequential(new VisionAlign());
//        addSequential(new SetHatchIntake(true));
//        addSequential(new RamseteCommand(rightRocketFromLoadingToFront,
//                drivetrain::updateRobotPose,
//                drivetrain.ramseteController,
//                KS,
//                KV,
//                KA,
//                drivetrain.differentialDriveKinematics,
//                encoderLeft::getRate,
//                encoderRight::getRate,
//                drivetrain.m_leftPIDController,
//                drivetrain.m_rightPIDController));
//        addSequential(new PointTurn(-35.681));
//        addSequential(new VisionAlign());
//        addSequential(new SetHatchIntake(false));
    }
}
