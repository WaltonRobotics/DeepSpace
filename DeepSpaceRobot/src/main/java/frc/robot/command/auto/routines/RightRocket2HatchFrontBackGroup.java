package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths;
import frc.robot.command.auto.SetHatchIntake;
import frc.robot.command.auto.VisionAlign;
import lib.Utils.RamseteCommand;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.*;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightRocket2HatchFrontBackGroup extends CommandGroup {
    public RightRocket2HatchFrontBackGroup() {
        addSequential(new RamseteCommand(Paths.RightRocketBackAndFrontLv1.generateToBackLv1(),
                drivetrain::updateRobotPoseStartBackwards,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1.2));
        addSequential(new SetHatchIntake(false));
        addSequential(new RamseteCommand(Paths.RightRocketBackAndFrontLv1.generateBackUpFromBackLv1(),
                drivetrain::updateRobotPoseStartBackwards,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new RamseteCommand(Paths.RightRocketBackAndFrontLv1.generateToLoadingStationFromBackLv1(),
                drivetrain::updateRobotPoseStartBackwards,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1.2));
        addSequential(new SetHatchIntake(true));
        addSequential(new RamseteCommand(Paths.RightRocketBackAndFrontLv1.generateToFrontLv1FromLoading(),
                drivetrain::updateRobotPoseStartBackwards,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1.2));
        addSequential(new SetHatchIntake(false));
    }
}
