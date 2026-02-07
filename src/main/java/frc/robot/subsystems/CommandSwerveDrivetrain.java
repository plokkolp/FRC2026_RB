package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

  private static final double kSimLoopPeriod = 0.005;

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private Supplier<Boolean> m_allianceSupplier;

  private final SwerveRequest.ApplyRobotSpeeds m_pathApply =
      new SwerveRequest.ApplyRobotSpeeds();

  // ===== SysId =====
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();

  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              s -> SignalLogger.writeString("SysIdTranslation_State", s.toString())),
          new SysIdRoutine.Mechanism(
              out -> setControl(m_translationCharacterization.withVolts(out)),
              null,
              this));

  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              s -> SignalLogger.writeString("SysIdSteer_State", s.toString())),
          new SysIdRoutine.Mechanism(
              v -> setControl(m_steerCharacterization.withVolts(v)),
              null,
              this));

  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(Math.PI / 6).per(Second),
              Volts.of(Math.PI),
              null,
              s -> SignalLogger.writeString("SysIdRotation_State", s.toString())),
          new SysIdRoutine.Mechanism(
              out -> {
                setControl(m_rotationCharacterization.withRotationalRate(out.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", out.in(Volts));
              },
              null,
              this));

  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  // ===== Debug =====
  private int m_resetPoseCount = 0;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) startSimThread();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) startSimThread();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStdDevs,
      Matrix<N3, N1> visionStdDevs,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStdDevs,
        visionStdDevs,
        modules);
    configurePathPlanner();
    if (Utils.isSimulation()) startSimThread();
  }

  // ===== Alliance =====
  public void setAllianceSupplier(Supplier<Boolean> allianceSupplier) {
    this.m_allianceSupplier = allianceSupplier;
  }

  // 這個會改 Pose heading（不是純 gyro 歸零），先保留但不要亂呼叫
  public void autoSeedFieldCentric() {
    double angle = (m_allianceSupplier != null && m_allianceSupplier.get()) ? 180.0 : 0.0;
    this.resetPose(new Pose2d(getState().Pose.getTranslation(), Rotation2d.fromDegrees(angle)));
  }

  /**
   * ★關鍵：提供「場地向」用的 heading。
   * 你目前的現象（車轉θ，正向轉2θ）幾乎等於 gyro 角度符號反了。
   * 所以這裡直接回傳 -yaw 來修正。
   */
  public Rotation2d getFieldHeading() {
    // 你 SmartDashboard 的 GyroYaw 顯示就是 degrees 量級（87、173那種）
    double yawDeg = getPigeon2().getYaw().getValueAsDouble();
    return Rotation2d.fromDegrees(-yawDeg); // ★修正 2θ 的核心
  }

  private void configurePathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner Config Error: " + e.getMessage(), e.getStackTrace());
      return;
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        speeds -> setControl(m_pathApply.withSpeeds(speeds)),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  @Override
  public void periodic() {
    // ===== Debug：只看真正會變的值 =====
    SmartDashboard.putNumber("DEBUG/GyroYaw_raw", getPigeon2().getYaw().getValueAsDouble());
    SmartDashboard.putNumber("DEBUG/FieldHeadingDeg_used", getFieldHeading().getDegrees());
    SmartDashboard.putNumber("DEBUG/PoseDeg", getState().Pose.getRotation().getDegrees());
    SmartDashboard.putBoolean("DEBUG/DSDisabled", DriverStation.isDisabled());
    SmartDashboard.putNumber("DEBUG/ResetPoseCount", m_resetPoseCount);
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getState().Speeds;
  }

  @Override
  public void resetPose(Pose2d pose) {
    m_resetPoseCount++;

    SmartDashboard.putNumber("DEBUG/LastReset_X", pose.getX());
    SmartDashboard.putNumber("DEBUG/LastReset_Y", pose.getY());
    SmartDashboard.putNumber("DEBUG/LastReset_Deg", pose.getRotation().getDegrees());

    System.out.println("[RESET POSE] " + pose);
    Thread.dumpStack();

    super.resetPose(pose);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> setControl(requestSupplier.get()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction d) {
    return m_sysIdRoutineToApply.quasistatic(d);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction d) {
    return m_sysIdRoutineToApply.dynamic(d);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();
    m_simNotifier =
        new Notifier(() -> {
          double now = Utils.getCurrentTimeSeconds();
          double dt = now - m_lastSimTime;
          m_lastSimTime = now;
          updateSimState(dt, RobotController.getBatteryVoltage());
        });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    super.addVisionMeasurement(pose, timestampSeconds);
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    super.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }
}
