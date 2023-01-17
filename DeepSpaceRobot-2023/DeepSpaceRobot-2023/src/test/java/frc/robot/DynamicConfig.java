/*
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.Before;
import org.junit.Test;

public class DynamicConfig {

  private NetworkTableInstance table;

  @Before
  public void initShuffleboard() {
    table = NetworkTableInstance.getDefault();
    table.setServer("localhost");
    table.startServer();
//    SendableChooser
  }

  @Test
  public void shuffleboardServer() throws InterruptedException {
    DynamicC dynamicC = new DynamicC();
    SmartDashboard.putData("Test", dynamicC);
    System.out.println(dynamicC.getTestValue());

    Thread.sleep(2000L);
    System.out.println(dynamicC.getTestValue());
    double testValue = SmartDashboard.getNumber("testValue", 2.0);
    System.out.println(testValue);
  }

  @Override
  public String toString() {
    return "DynamicConfig{" +
        "table=" + table +
        '}';
  }
}
*/
