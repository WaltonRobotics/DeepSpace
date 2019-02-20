package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class JoysctickButtonEnhanced extends JoystickButton {

  private boolean value;
  private boolean toggleValue;

  /**
   * Create a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoysctickButtonEnhanced(GenericHID joystick, int buttonNumber) {
    super(joystick, buttonNumber);

    whenActive(new InstantCommand(() -> value = true));
    whenInactive(new InstantCommand(() -> value = false));
    whenPressed(new InstantCommand(() -> toggleValue = !toggleValue));
  }

  @Override
  public boolean get() {
    return value;
  }

  public boolean getToggle() {
    return toggleValue;
  }
}
