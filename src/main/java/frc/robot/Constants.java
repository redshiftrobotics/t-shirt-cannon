package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double LOOP_PERIOD_SECONDS = Robot.defaultPeriodSecs; // 0.02

  public static final boolean TUNING_MODE = false;

  private static RobotType robotType = RobotType.CANNON_BOT;

  public static final Alert wrongRobotTypeAlertReal =
      new Alert("Invalid robot selected, using cannon robot as default.", Alert.AlertType.kWarning);

  private static final Alert wrongRobotTypeAlertSim =
      new Alert(
          "Invalid robot selected for simulation robot, using simulation robot as default.",
          AlertType.kError);

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIM_BOT) {
      wrongRobotTypeAlertReal.set(true);
      robotType = RobotType.CANNON_BOT;
    }
    if (RobotBase.isSimulation() && robotType != RobotType.SIM_BOT) {
      wrongRobotTypeAlertSim.set(true);
      robotType = RobotType.SIM_BOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (getRobot()) {
      case CANNON_BOT, TEST_BOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM_BOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIM_BOT,
    CANNON_BOT,
    TEST_BOT,
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIM_BOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }
}
