package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.TeleopDriveController;

/** Command to drive when new driver is trying robot out */
public class SafeDriveMode extends Command {

  public static final double TRANSLATION_SPEED_SCALE = 0.5;
  public static final double ROTATION_SPEED_SCALE = 0.5;

  private final Drive drive;
  private final TeleopDriveController controller;
  private final boolean fieldRelative;
  private final Translation2d bottomLeftAllowedCorner;
  private final Translation2d topRightAllowedCorner;

  private final Timer breakModeTimer;

  public SafeDriveMode(Drive drive, TeleopDriveController controller, boolean fieldRelative) {
    this(drive, controller, fieldRelative, null, null);
  }

  public SafeDriveMode(
      Drive drive,
      TeleopDriveController controller,
      boolean fieldRelative,
      Translation2d bottomLeftAllowedCorner,
      Translation2d topRightAllowedCorner) {
    this.drive = drive;
    this.controller = controller;

    this.fieldRelative = fieldRelative;

    this.bottomLeftAllowedCorner = bottomLeftAllowedCorner;
    this.topRightAllowedCorner = topRightAllowedCorner;

    this.breakModeTimer = new Timer();

    addRequirements(drive);
  }

  @Override
  public void execute() {
    boolean isInSafeBox = isInSafeBox();
    boolean isBrakeMode = drive.getMotorBrakeOnCoastModeEnabled();
    boolean enabled = RobotState.isEnabled();
    boolean teleop = RobotState.isTeleop();

    SmartDashboard.putBoolean("IsInSafeBox?", isInSafeBox);
    SmartDashboard.putBoolean("CanBePushed?", !(isBrakeMode || enabled));

    if (isInSafeBox) {
      if (!isBrakeMode) {
        drive.setMotorBrakeOnCoastModeEnabled(true);
      }

      breakModeTimer.restart();

      if (enabled && teleop) {
        Translation2d translation =
            controller.getTranslationMetersPerSecond().times(TRANSLATION_SPEED_SCALE);
        double rotation = controller.getOmegaRadiansPerSecond() * ROTATION_SPEED_SCALE;
        drive.setRobotSpeeds(
            new ChassisSpeeds(translation.getX(), translation.getY(), rotation), fieldRelative);
      }
    } else {
      if (enabled) {
        drive.stopUsingForwardArrangement();
        if (breakModeTimer.hasElapsed(3) && isBrakeMode) {
          drive.setMotorBrakeOnCoastModeEnabled(false);
        }
      } else {
        drive.stop();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  private boolean isInSafeBox() {
    Pose2d currentPose = drive.getPose();

    return (bottomLeftAllowedCorner != null
            && bottomLeftAllowedCorner.getX() < currentPose.getX()
            && bottomLeftAllowedCorner.getY() < currentPose.getY())
        && (topRightAllowedCorner != null
            && topRightAllowedCorner.getX() > currentPose.getX()
            && topRightAllowedCorner.getY() > currentPose.getY());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
