package frc.robot.dynamicCondig;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

class Variable {

  public final String name;
  public final DynamicConfig instance;
  public Method accessor;
  public Method mutator;
  private Setter.ValueType valueType;

  public Variable(DynamicConfig instance, String name) {
    this.instance = instance;
    this.name = name;
  }

  @Override
  public String toString() {
    return "Variable{" +
        "name='" + name + '\'' +
        ", instance=" + instance +
        ", accessor=" + accessor +
        ", mutator=" + mutator +
        ", valueType=" + valueType +
        '}';
  }

  public Setter.ValueType getValueType() {
    return valueType;
  }

  public Object getAccessor() {
    System.out.println("Hello");

    try {
      return accessor.invoke(instance);
    } catch (IllegalAccessException e) {
      e.printStackTrace();
    } catch (InvocationTargetException e) {
      e.printStackTrace();
    }
    return null;
  }

  public void setAccessor(Method accessor) {
    if (accessor.getParameterCount() != 0) {
      throw new IllegalArgumentException("The mutator method must not need a parameter");
    }

    this.accessor = accessor;
  }

  public void getMutator(Object value) {
    try {
      mutator.invoke(instance, value);
    } catch (IllegalAccessException e) {
      e.printStackTrace();
    } catch (InvocationTargetException e) {
      e.printStackTrace();
    }
  }

  public void setMutator(Method mutator, Setter.ValueType valueType) {
    if (mutator != null && mutator.getParameterCount() != 1
        && mutator.getParameterTypes()[0].isInstance(valueType.getDefiningClass())) {
      throw new IllegalArgumentException("The mutator method must only need a single parameter");
    }

    this.mutator = mutator;
    this.valueType = valueType;
  }
}