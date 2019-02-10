package frc.robot;

import frc.robot.util.CrashDump;
import org.junit.Test;

import java.io.File;

import static org.junit.Assert.assertTrue;

public class CrashDumpTest {

    @Test
    public void crashDumpFileExistsTest() {
        CrashDump.setCrashDumpDefaultFilePath("C:\\Logs\\CrashDump.txt");

        CrashDump.logThrowableCrash(new Throwable("[IGNORE]: Crash dump file exists test."));

        File tmpDir = new File(CrashDump.CRASH_DUMP_DEFAULT_FILE_PATH);

        assertTrue(tmpDir.exists());
    }

}