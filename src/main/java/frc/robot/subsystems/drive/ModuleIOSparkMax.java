package frc.robot.subsystems.drive;

import static frc.robot.utility.SparkUtil.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import frc.robot.utility.SparkUtil;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and CANCoder absolute encoder
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad". Update these values in DriveConstants.
 */
public class ModuleIOSparkMax implements ModuleIO {

  // Hardware
  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final CANcoder cancoder;

  // Built In Encoders
  private final RelativeEncoder driveRelativeEncoder;
  private final RelativeEncoder turnRelativeEncoder;

  // Build In PID
  private final SparkClosedLoopController driveFeedback;
  private final SparkClosedLoopController turnFeedback;

  // Odometry queues
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Absolute encoder signal
  private final StatusSignal<Angle> turnAbsolutePosition;

  // Connection debouncer
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSparkMax(ModuleConfig config) {

    // Create motor controllers
    driveSpark = new SparkMax(config.driveID(), MotorType.kBrushless);
    turnSpark = new SparkMax(config.turnID(), MotorType.kBrushless);

    cancoder = new CANcoder(config.absoluteEncoderChannel());

    // Relative Encoder
    driveRelativeEncoder = driveSpark.getEncoder();
    turnRelativeEncoder = turnSpark.getEncoder();

    // PID
    driveFeedback = driveSpark.getClosedLoopController();
    turnFeedback = turnSpark.getClosedLoopController();

    // Cancoder config
    MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
    magnetSensorConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    magnetSensorConfig.AbsoluteSensorDiscontinuityPoint = 1.0;
    magnetSensorConfig.MagnetOffset = config.absoluteEncoderOffset().getRotations();
    cancoder.getConfigurator().apply(magnetSensorConfig);

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnAbsolutePosition.setUpdateFrequency(DriveConstants.odometryFrequencyHertz);

    // Configure drive motor
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(1 / DriveConstants.driveReduction)
        .velocityConversionFactor(1 / DriveConstants.driveReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0.0, 0.0, 0.0, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequencyHertz))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveRelativeEncoder.setPosition(0.0));

    // Configure turn motor
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(config.turnMotorInverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(1 / DriveConstants.turnReduction)
        .velocityConversionFactor(1 / DriveConstants.turnReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 1)
        .pidf(0.0, 0.0, 0.0, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequencyHertz))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    turnRelativeEncoder.setPosition(turnAbsolutePosition.getValueAsDouble());

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(driveSpark, driveRelativeEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(turnSpark, turnRelativeEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // --- Drive ---
    SparkUtil.clearStickyFault();
    ifOk(
        driveSpark,
        driveRelativeEncoder::getPosition,
        value -> inputs.drivePositionRad = Units.rotationsToRadians(value));
    ifOk(
        driveSpark,
        driveRelativeEncoder::getVelocity,
        value -> inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
    ifOk(
        driveSpark,
        () -> driveSpark.getAppliedOutput() * driveSpark.getBusVoltage(),
        value -> inputs.driveAppliedVolts = value);
    ifOk(driveSpark, driveSpark::getOutputCurrent, value -> inputs.driveSupplyCurrentAmps = value);
    inputs.driveMotorConnected = driveConnectedDebounce.calculate(!SparkUtil.hasStickyFault());

    // --- Turn ---
    SparkUtil.clearStickyFault();

    ifOk(
        turnSpark,
        turnRelativeEncoder::getPosition,
        value -> inputs.turnPosition = Rotation2d.fromRotations(value));
    ifOk(
        turnSpark,
        turnRelativeEncoder::getVelocity,
        value -> inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
    ifOk(
        turnSpark,
        () -> turnSpark.getAppliedOutput() * turnSpark.getBusVoltage(),
        value -> inputs.turnAppliedVolts = value);

    ifOk(turnSpark, turnSpark::getOutputCurrent, value -> inputs.turnSupplyCurrentAmps = value);

    inputs.turnMotorConnected = turnConnectedDebounce.calculate(!SparkUtil.hasStickyFault());

    // --- Absolute Encoder ---
    inputs.turnAbsoluteEncoderConnected = cancoder.isConnected();
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.refresh().getValueAsDouble());

    // --- Odometry ---
    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

    // --- Clear queues after use ---
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSpark.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSpark.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double velocityRadsPerSec, double feedForwardVoltage) {
    driveFeedback.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadsPerSec),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedForwardVoltage,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(double angleRads) {
    turnFeedback.setReference(
        Units.radiansToRotations(angleRads), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.closedLoop.pidf(kP, kI, kD, 0.0);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.closedLoop.pidf(kP, kI, kD, 0.0);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void stop() {
    driveSpark.stopMotor();
    turnSpark.stopMotor();
  }
}
