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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  private final SwerveRequest.ApplyRobotSpeeds m_pathApply = new SwerveRequest.ApplyRobotSpeeds();
  private final Field2d field = new Field2d();

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

  private int m_resetPoseCount = 0;

  // =========================
  // 重要：WPILib kinematics/odometry 座標系
  // +X 向前、+Y 向左
  // 你原本把 +Y 寫成右邊，會導致里程計/角度怪
  // =========================
  private static final Translation2d kFL =
      new Translation2d(Units.inchesToMeters(+14.75), Units.inchesToMeters(+12.75));
  private static final Translation2d kFR =
      new Translation2d(Units.inchesToMeters(+14.75), Units.inchesToMeters(-12.75));
  private static final Translation2d kBL =
      new Translation2d(Units.inchesToMeters(-14.75), Units.inchesToMeters(+12.75));
  private static final Translation2d kBR =
      new Translation2d(Units.inchesToMeters(-14.75), Units.inchesToMeters(-12.75));

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(kFL, kFR, kBL, kBR);

  private static final Matrix<N3, N1> kStateStdDevs =
      VecBuilder.fill(0.25, 0.25, Math.toRadians(8.0));

  private static final Matrix<N3, N1> kVisionStdDevs =
      VecBuilder.fill(0.08, 0.08, Math.toRadians(4.0));

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          Rotation2d.fromDegrees(0),
          new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
          },
          new Pose2d(),
          kStateStdDevs,
          kVisionStdDevs);

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
    super(drivetrainConstants, odometryUpdateFrequency, odometryStdDevs, visionStdDevs, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) startSimThread();
  }

  // ===== Alliance =====
  public void setAllianceSupplier(Supplier<Boolean> allianceSupplier) {
    this.m_allianceSupplier = allianceSupplier;
  }

  // =========================
  // 里程計角度（你要的「加負號」就在這裡）
  // WPILib 慣例：逆時針(CCW)為正
  // 常見 Pigeon2：順時針為正 -> 這裡反號一次統一
  // =========================
  private Rotation2d getOdometryHeading() {
    return Rotation2d.fromDegrees(-getPigeon2().getYaw().getValueAsDouble());
  }

  // 給手控 FieldRelative 用（跟里程計同一套）
  public Rotation2d getFieldHeading() {
    return getOdometryHeading();
  }

  // RobotContainer 的 LB 建議呼叫這個
  public void seedFieldCentric() {
    getPigeon2().setYaw(0);
    Pose2d cur = getPose();
    m_poseEstimator.resetPosition(
        getOdometryHeading(),
        getState().ModulePositions,
        new Pose2d(cur.getTranslation(), Rotation2d.fromDegrees(0)));
  }

  public void autoSeedFieldCentric() {
    double angle = (m_allianceSupplier != null && m_allianceSupplier.get()) ? 180.0 : 0.0;

    // 因為 getOdometryHeading() 會反號，所以 setYaw 也反號一次讓「里程計角度」變成 angle
    getPigeon2().setYaw(-angle);

    Pose2d cur = getPose();
    Pose2d target = new Pose2d(cur.getTranslation(), Rotation2d.fromDegrees(angle));
    m_poseEstimator.resetPosition(getOdometryHeading(), getState().ModulePositions, target);
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
        speeds -> setControl(m_pathApply.withSpeeds(speeds)), // PathPlanner 輸出是 robot-relative，別再轉換
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  @Override
  public void periodic() {
    // 用「真陀螺儀角度(含負號修正)」更新里程計
    m_poseEstimator.update(getOdometryHeading(), getState().ModulePositions);

    SmartDashboard.putNumber("DEBUG/GyroYaw_raw", getPigeon2().getYaw().getValueAsDouble());
    SmartDashboard.putNumber("DEBUG/OdomHeading_used", getOdometryHeading().getDegrees());

    SmartDashboard.putNumber("DEBUG/PoseDeg_phoenix", getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("DEBUG/PoseDeg_est", getPose().getRotation().getDegrees());

    SmartDashboard.putBoolean("DEBUG/DSDisabled", DriverStation.isDisabled());
    SmartDashboard.putNumber("DEBUG/ResetPoseCount", m_resetPoseCount);

    SmartDashboard.putNumber("Pose/X", getPose().getX());
    SmartDashboard.putNumber("Pose/Y", getPose().getY());

    field.setRobotPose(getPose());
    SmartDashboard.putData("field", field);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
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

    // 只重設 estimator，避免兩套里程計互打
    m_poseEstimator.resetPosition(getOdometryHeading(), getState().ModulePositions, pose);
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
    m_poseEstimator.addVisionMeasurement(pose, timestampSeconds);
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    m_poseEstimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }
}