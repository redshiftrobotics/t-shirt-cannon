package frc.robot.subsystems.cannon;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FiringTube extends SubsystemBase {
  private final CannonIO io;
  private final CannonIOInputsAutoLogged inputs = new CannonIOInputsAutoLogged();

  private final Timer fireTimer;

  public FiringTube(CannonIO io) {
    this.io = io;
    fireTimer = new Timer();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FiringTube", inputs);

    if (fireTimer.hasElapsed(CannonConstants.FIRE_TUBE_OPEN_TIME_SECONDS)) {
      io.close();
      fireTimer.stop();
    }
  }

  /**
   * Opens the firing tube and starts the timer to close it.
   */
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
