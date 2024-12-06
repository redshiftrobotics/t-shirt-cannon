package frc.robot.subsystems.cannon;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class FiringTube {
  private final CannonIO io;
  private final CannonIOInputsAutoLogged inputs = new CannonIOInputsAutoLogged();

  private final String name;

  private final Timer fireTimer;

  /** Creates a new FiringTube subsystem */
  public FiringTube(CannonIO io, String name) {
    this.io = io;
    this.name = name;
    fireTimer = new Timer();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FiringTube" + name, inputs);

    if (fireTimer.hasElapsed(CannonConstants.FIRE_TUBE_OPEN_TIME_SECONDS)) {
      io.close();
      fireTimer.stop();
    }
  }

  /** Opens the firing tube and starts the timer to close it. */
  public void fire() {
    io.open();
    fireTimer.restart();
  }

  /**
   * Returns whether the firing tube is open.
   *
   * @return true if the firing tube is open, false otherwise
   */
  public boolean isOpen() {
    return inputs.isOpen;
  }
}
