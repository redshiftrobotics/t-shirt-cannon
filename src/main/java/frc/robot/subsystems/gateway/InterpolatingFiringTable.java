package frc.robot.subsystems.gateway;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingFiringTable {

  private InterpolatingFiringTable() {}

  public record ShotParameter(double psi) {
    public ShotParameter interpolate(ShotParameter other, double t) {
      return new ShotParameter(psi + (t * (other.psi - psi)));
    }
  }

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(Map.ofEntries(entry(Units.inchesToMeters(50), new ShotParameter(100))));

  public static ShotParameter get(double distanceToTarget) {
    Entry<Double, ShotParameter> ceil = table.ceilingEntry(distanceToTarget);
    Entry<Double, ShotParameter> floor = table.floorEntry(distanceToTarget);
    if (ceil == null) return floor.getValue();
    if (floor == null) return ceil.getValue();
    if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
    return floor
        .getValue()
        .interpolate(
            ceil.getValue(),
            (distanceToTarget - floor.getKey()) / (ceil.getKey() - floor.getKey()));
  }
}