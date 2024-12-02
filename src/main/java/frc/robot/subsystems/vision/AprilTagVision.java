package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.Camera.VisionResultStatus;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private final Camera[] cameras;

  private Supplier<Pose2d> robotPoseSupplier = null;
  private List<Consumer<TimestampedRobotPoseEstimate>> timestampRobotPoseEstimateConsumers =
      new ArrayList<>();

  public AprilTagVision(CameraIO... camerasIO) {
    this.cameras =
        Arrays.stream(camerasIO)
            .map(io -> new Camera(io, FieldConstants.APRIL_TAG_FIELD_LAYOUT))
            .toArray(Camera[]::new);
  }

  @Override
  public void periodic() {
    cameras().forEach(Camera::periodic);

    for (Camera camera : cameras) {
      String root = "Vision/" + camera.getCameraName();

      if (!camera.hasNewData()) {
        Logger.recordOutput(root + "/tagsUsedPositions", new Pose3d[] {});
        continue;
      }

      VisionResultStatus status =
          robotPoseSupplier == null
              ? camera.getStatus()
              : camera.getStatus(robotPoseSupplier.get());

      // Get Data
      TimestampedRobotPoseEstimate visionEstimate =
          new TimestampedRobotPoseEstimate(
              camera.getEstimatedRobotPose(),
              camera.getTimestampSeconds(),
              camera.getStandardDeviations(),
              status);

      // Logging

      Logger.recordOutput(root + "/tagsUsedPositions", camera.getTagPositionsOnField());

      Logger.recordOutput(root + "/positionEstimate", visionEstimate.robotPose());

      Logger.recordOutput(root + "/status", visionEstimate.status);
      Logger.recordOutput(root + "/statusIsSuccess", visionEstimate.status.isSuccess());

      // Give consumers estimate

      for (Consumer<TimestampedRobotPoseEstimate> consumer : timestampRobotPoseEstimateConsumers) {
        consumer.accept(visionEstimate);
      }
    }
  }

  public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
  }

  public void addVisionEstimateConsumer(
      Consumer<TimestampedRobotPoseEstimate> timestampRobotPoseEstimateConsumer) {
    timestampRobotPoseEstimateConsumers.add(timestampRobotPoseEstimateConsumer);
  }

  private Stream<Camera> cameras() {
    return Arrays.stream(cameras);
  }

  @Override
  public String toString() {
    return String.format(
        "%s(%s)",
        getClass().getName(),
        Arrays.stream(cameras).map(Camera::getCameraName).collect(Collectors.joining(", ")));
  }

  public record TimestampedRobotPoseEstimate(
      Pose3d robotPose,
      double timestampSeconds,
      Matrix<N3, N1> standardDeviations,
      VisionResultStatus status) {

    public Pose2d robotPose2d() {
      return robotPose.toPose2d();
    }

    public boolean isSuccess() {
      return status.isSuccess();
    }
  }
}
