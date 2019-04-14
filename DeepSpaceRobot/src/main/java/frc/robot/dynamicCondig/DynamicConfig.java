package frc.robot.dynamicCondig;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicInteger;

public class DynamicConfig extends SendableBase {

  private static final AtomicInteger instances = new AtomicInteger();
  private final Map<String, Variable> methods = new HashMap<>(2);

  public DynamicConfig() {
    super(false);

    for (Method method : getClass().getMethods()) {
      if (method.isAnnotationPresent(Getter.class)) {
        Getter annotation = method.getAnnotation(Getter.class);
        if (methods.containsKey(annotation.name())) {
          methods.get(annotation.name()).setAccessor(method);
        } else {
          Variable variable = new Variable(this, annotation.name());
          variable.setAccessor(method);
          methods.put(annotation.name(), variable);
        }
      } else if (method.isAnnotationPresent(Setter.class)) {
        Setter annotation = method.getAnnotation(Setter.class);
        if (methods.containsKey(annotation.name())) {
          methods.get(annotation.name()).setMutator(method, annotation.value());
        } else {
          Variable variable = new Variable(this, annotation.name());
          variable.setMutator(method, annotation.value());
          methods.put(annotation.name(), variable);
        }
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
//    SmartDashboard.putString()
    builder.setSmartDashboardType("RobotPreferences");
    builder.getEntry(".instance").setDouble(instances.getAndIncrement());
    builder.getEntry(".name").setString("Cheese");

//    SendableChooser
    for (Entry<String, Variable> entry : methods.entrySet()) {
      if (entry.getValue().getValueType() == Setter.ValueType.BOOLEAN) {
        builder.addBooleanProperty(
            entry.getKey(),
            () -> (Boolean) entry.getValue().getAccessor(),
            b -> entry.getValue().getMutator(b)
        );
      } else if (entry.getValue().getValueType() == Setter.ValueType.NUMBER) {
        builder.addDoubleProperty(
            entry.getKey(),
            () -> (Double) entry.getValue().getAccessor(),
            b -> entry.getValue().getMutator(b)
        );
      } else {
        builder.addStringProperty(
            entry.getKey(),
            () -> String.valueOf(entry.getValue().getAccessor()),
            b -> entry.getValue().getMutator(String.valueOf(b))
        );
      }
    }
  }

  @Override
  public String toString() {
    return "DynamicConfig{" +
        "methods=" + methods +
        "} " + super.toString();
  }
}