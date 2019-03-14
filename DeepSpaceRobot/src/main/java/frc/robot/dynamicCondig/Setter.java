package frc.robot.dynamicCondig;


import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface Setter {

  ValueType value();

  String name();

  enum ValueType {
    BOOLEAN(Boolean.class), STRING(String.class), NUMBER(Number.class);

    private final Class aClass;

    ValueType(Class aClass) {

      this.aClass = aClass;
    }

    public Class getDefiningClass() {
      return aClass;
    }
  }
}
