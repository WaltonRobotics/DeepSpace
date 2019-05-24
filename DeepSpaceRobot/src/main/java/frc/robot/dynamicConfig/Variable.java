package frc.robot.dynamicConfig;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

class Variable {

    private final String name;
    private final DynamicConfig instance;
    private Method accessor;
    private Method mutator;
    private Setter.ValueType valueType;

    Variable(DynamicConfig instance, String name) {
        this.instance = instance;
        this.name = name;
    }

    Setter.ValueType getValueType() {
        return valueType;
    }

    Object getAccessor() {
        try {
            return accessor.invoke(instance);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (InvocationTargetException e) {
            e.printStackTrace();
        }
        return null;
    }

    void setAccessor(Method accessor) {
        if (accessor.getParameterCount() != 0) {
            throw new IllegalArgumentException("The mutator method must not need a parameter");
        }

        this.accessor = accessor;
    }

    void getMutator(Object value) {
        try {
            mutator.invoke(instance, value);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (InvocationTargetException e) {
            e.printStackTrace();
        }
    }

    void setMutator(Method mutator, Setter.ValueType valueType) {
        if ((mutator != null) && (mutator.getParameterCount() != 1)
                && mutator.getParameterTypes()[0].isInstance(valueType.getDefiningClass())) {
            throw new IllegalArgumentException("The mutator method must only need a single parameter");
        }

        this.mutator = mutator;
        this.valueType = valueType;
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
}