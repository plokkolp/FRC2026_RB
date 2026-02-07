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

// ===== Vision 相關先全部註解掉（暫時不用）=====
// import frc.robot.subsystems.Vision.VisionUpdate;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

  private static final double kSimLoopPeriod = 0.005;

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  private boolean m_hasAppliedOperatorPerspective = false;

  // ===== Vision 相關先全部註解掉（暫時不用）=====
  // private final Vision m_vision;
  // private boolean m_hasVisionInitializedPose = false;
  // private boolean m_allowVisionReset = true;

  private Supplier<Boolean> m_allianceSupplier;

  private final SwerveRequest.ApplyRobotSpeeds m_pathApply =
      new SwerveRequest.ApplyRobotSpeeds();

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

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {

    super(drivetrainConstants, modules);

    // ===== Vision 相關先全部註解掉（暫時不用）=====
    // m_vision = new Vision(this);

    configurePathPlanner();

    if (Utils.isSimulation()) startSimThread();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {

    super(drivetrainConstants, odometryUpdateFrequency, modules);

    // ===== Vision 相關先全部註解掉（暫時不用）=====
    // m_vision = new Vision(this);

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

    // ===== Vision 相關先全部註解掉（暫時不用）=====
    // m_vision = new Vision(this);

    configurePathPlanner();

    if (Utils.isSimulation()) startSimThread();
  }

  // 航向
  public void setAllianceSupplier(Supplier<Boolean> allianceSupplier) {
    this.m_allianceSupplier = allianceSupplier;
  }

  public void autoSeedFieldCentric() {
    double angle = (m_allianceSupplier != null && m_allianceSupplier.get()) ? 180.0 : 0.0;
    this.resetPose(new Pose2d(getState().Pose.getTranslation(), Rotation2d.fromDegrees(angle)));
  }

  private void configurePathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError(
          "PathPlanner Config Error: " + e.getMessage(),
          e.getStackTrace());
      return;
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        speeds -> {
          setControl(m_pathApply.withSpeeds(speeds));
        },
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  // ===== Vision 相關先全部註解掉（暫時不用）=====
  /*
  public void handleVisionUpdate(VisionUpdate update) {
    switch (update.action) {
      case INITIALIZE:
        if (DriverStation.isDisabled()
            && m_allowVisionReset
            && !m_hasVisionInitializedPose) {
          resetPose(update.pose);
          m_hasVisionInitializedPose = true;
        }
        break;

      case FUSE:
        addVisionMeasurement(
            update.pose,
            update.timestamp,
            VecBuilder.fill(
                update.stdX,
                update.stdY,
                update.stdTheta));
        break;

      default:
        break;
    }
  }
  */

  @Override
  public void periodic() {

    // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    //   DriverStation.getAlliance().ifPresent(alliance -> {
    //     setOperatorPerspectiveForward(
    //         alliance == Alliance.Red
    //             ? kRedAlliancePerspectiveRotation
    //             : kBlueAlliancePerspectiveRotation);
    //     m_hasAppliedOperatorPerspective = true;
    //   });
    // }
    if (!m_hasAppliedOperatorPerspective) {
    DriverStation.getAlliance().ifPresent(alliance -> {
      setOperatorPerspectiveForward(
          alliance == Alliance.Red
              ? kRedAlliancePerspectiveRotation
              : kBlueAlliancePerspectiveRotation);
      m_hasAppliedOperatorPerspective = true;
    });
  }

    // ===== Vision 相關先全部註解掉（暫時不用）=====
    /*
    if (DriverStation.isDisabled()) {
      m_allowVisionReset = true;
    } else {
      m_allowVisionReset = false;
    }

    m_vision.periodic();
    */
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getState().Speeds;
  }

  @Override
public void resetPose(Pose2d pose) {
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
  public void addVisionMeasurement(
      Pose2d pose,
      double timestampSeconds,
      Matrix<N3, N1> stdDevs) {
    super.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }
}
