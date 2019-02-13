package frc.robot;

import frc.robot.util.CrashDump;
import org.junit.Test;

import java.io.File;

import static org.junit.Assert.assertTrue;

public class CrashDumpTest {

    @Test
    public void crashDumpFileExistsTest() {
        CrashDump.setCrashDumpFilePath("src/test/java/frc/robot/CrashDump.txt");

        CrashDump.logThrowableCrash(new Throwable("[IGNORE]: Crash dump file exists test."));

        File tmpDir = new File(CrashDump.getCrashDumpFilePath());

        assertTrue(tmpDir.exists());
    }

}