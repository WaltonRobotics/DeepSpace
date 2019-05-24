package frc.robot;

import frc.robot.dynamicConfig.Getter;
import frc.robot.dynamicConfig.Setter;
import frc.robot.dynamicConfig.Setter.ValueType;

public class DynamicC extends frc.robot.dynamicConfig.DynamicConfig {

  public double testValue;

  @Getter(name = "testValue")
  public double getTestValue() {
    return testValue;
  }

  @Setter(name = "testValue", value = ValueType.NUMBER)
  public void setTestValue(double value) {
    testValue = value;
  }

  @Override
  public String toString() {
    return "DynamicC{" +
        "testValue=" + testValue +
        "} " + super.toString();
  }
}
