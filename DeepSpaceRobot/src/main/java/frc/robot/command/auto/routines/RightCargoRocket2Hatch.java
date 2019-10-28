package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths;
import frc.robot.command.auto.SetHatchIntake;
import frc.robot.command.auto.VisionAlign;
import lib.Utils.RamseteCommand;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Config.SmartMotionConstants.KA;
import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightCargoRocket2Hatch extends CommandGroup {
    public RightCargoRocket2Hatch() {
        addSequential(new RamseteCommand(Paths.Right2HatchCargoRocket.generateToCargo1(),
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
        addSequential(new VisionAlign(1.5));
        addSequential(new SetHatchIntake(false));
        addSequential(new RamseteCommand(Paths.Right2HatchCargoRocket.generateCargo1BackUp(),
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
        addSequential(new RamseteCommand(Paths.Right2HatchCargoRocket.generateCargo1ToLoadingStation(),
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
        addSequential(new VisionAlign(1.8));
        addSequential(new SetHatchIntake(true));
        addSequential(new RamseteCommand(Paths.Right2HatchCargoRocket.generateToFrontLv1FromLoading(),
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
        addSequential(new VisionAlign(3));
        addSequential(new SetHatchIntake(false));
    }
}
