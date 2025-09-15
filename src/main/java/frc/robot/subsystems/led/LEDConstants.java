package frc.robot.subsystems.led;

import frc.robot.Constants;

public class LEDConstants {
  public static final BlinkenLEDPattern OFF = BlinkenLEDPattern.BLACK;

  public static final int[] PWM_PORTS =
      switch (Constants.getRobot()) {
        case CANNON_BOT -> new int[] {0};
        default -> new int[] {0};
      };
}
