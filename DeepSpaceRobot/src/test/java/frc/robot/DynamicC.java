package frc.robot;

import frc.robot.dynamicCondig.Getter;
import frc.robot.dynamicCondig.Setter;
import frc.robot.dynamicCondig.Setter.ValueType;

public class DynamicC extends frc.robot.dynamicCondig.DynamicConfig {

  public double testValue;

  @Getter(name = "testValue")
  public double getTestValue() {
    return testValue;
  }

  @Setter(name = "testValue", value = ValueType.NUMBER)
  public void setTestValue(double value) {
    testValue = value;
  }
}