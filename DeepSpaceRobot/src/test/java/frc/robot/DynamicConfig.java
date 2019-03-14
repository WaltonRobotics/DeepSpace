package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.Assert;
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
    System.out.println("Name: " + dynamicC.getName());
    SmartDashboard.putData("Test", dynamicC);
    System.out.println(dynamicC.getTestValue());

    Thread.sleep(2000);
    System.out.println(dynamicC.getTestValue());
    double testValue = SmartDashboard.getNumber("testValue", 2);
    System.out.println(testValue);
    Assert.fail();

  }
}

