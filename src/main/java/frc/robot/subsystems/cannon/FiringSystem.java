package frc.robot.subsystems.cannon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.stream.Stream;

public class FiringSystem extends SubsystemBase {

  public final FiringTube[] tubes;

  /** Creates a new FiringSystem subsystem */
  public FiringSystem(FiringTube... tubes) {
    this.tubes = tubes;
  }

  @Override
  public void periodic() {
    tubes().forEach(FiringTube::periodic);
  }

  public boolean isOpen() {
    return tubes().allMatch(FiringTube::isOpen);
  }

  private Stream<FiringTube> tubes() {
    return Arrays.stream(tubes);
  }
}
