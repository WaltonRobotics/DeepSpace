package frc.robot;

import edu.wpi.first.wpiutil.RuntimeDetector;
import edu.wpi.first.wpiutil.RuntimeLoader;
import frc.robot.Command.teleop.Drive;
import frc.robot.Subsystem.Drivetrain;
import org.junit.Assert;
import org.junit.Before;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

import static frc.robot.Robot.drivetrain;

public class MotorTest
{
    private static void loadNativeLibraries() {
        Path path = Paths.get(RuntimeLoader.getDefaultExtractionRoot(), RuntimeDetector.getPlatformPath());

        File file = path.toFile();

        for (File nativeLibrary : Objects.requireNonNull(file.listFiles())) {
            System.out.printf("Loading: %s from %s%n", nativeLibrary.getName(), nativeLibrary.getAbsolutePath());
            System.load(nativeLibrary.getAbsolutePath());
        }
    }
    @Before
    public void init(){
        loadNativeLibraries();
    }
    @Test
    public void move()
    {
        int speeds = 5;
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.setSpeeds(speeds, speeds);
        Assert.assertEquals(speeds, drivetrain.getMaxVelocity(), 0);
    }
}
