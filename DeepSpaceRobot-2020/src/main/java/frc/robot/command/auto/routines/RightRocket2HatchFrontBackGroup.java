package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Paths.RightRocketBackAndFrontLv1;
import frc.robot.command.auto.RamseteTrackingCommand;
import frc.robot.command.auto.VisionAlign;

import static frc.robot.Robot.*;

public class RightRocket2HatchFrontBackGroup extends CommandGroup {

    public RightRocket2HatchFrontBackGroup() {

        Trajectory toFrontLv1 = RightRocketBackAndFrontLv1.generateToFrontLv1();
        Trajectory backUpLv1 = RightRocketBackAndFrontLv1.generateBackUpFrontLv1();
        Trajectory toLoadingStationfromLv1 = RightRocketBackAndFrontLv1.generateToLoadingStation();
        Trajectory generateToBackLv1FromLoadingStation = RightRocketBackAndFrontLv1.generateToBackLv1FromLoadingStation();

        addSequential(new RamseteTrackingCommand(toFrontLv1, true));
        addSequential(new VisionAlign(1));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(false)));
        addSequential(new RamseteTrackingCommand(backUpLv1, true));
        addSequential(new RamseteTrackingCommand(toLoadingStationfromLv1, true));
        addSequential(new VisionAlign(1));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(true)));
        addSequential(new RamseteTrackingCommand(generateToBackLv1FromLoadingStation, true));
        addSequential(new VisionAlign(1.1));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(false)));
    }
}
