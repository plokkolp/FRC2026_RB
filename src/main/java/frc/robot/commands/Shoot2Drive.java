package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterLookup;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class Shoot2Drive extends Command {

  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;
  private final XboxController controller;
  private final XboxController drive;

  private final SwerveRequest.ApplyRobotSpeeds driveRequest =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private static final double kYawP = 0.0045;
  private static final double kYawD = 0.0010;
  private static final double kYawMaxOut = 0.6;
  private static final double kYawTolDeg = 2.0;
  private static final double kMaxValidTxDeg = 30.0;

  private static final double kTrainDuty = -0.7;
  private static final double kIntakeDuty = -0.7;
  private static final double kFireTrig = 0.3;

  private static final boolean kGateByRPM = false;
  private static final double kRpmTol = 120.0;
  private static final double kSpinupMinTime = 0.2;

  private static final double kManualDeadband = 0.1;
  private static final double kManualScale = 0.06;

  private static final double kMaxOmegaRadPerSec = 1.0 * Math.PI;

  private static final double kShotRpmOffset = 180.0;

  private double startTime;
  private double lastTargetRpm = 2200.0;
  private double lastTargetPitchRot = -0.45;
  private double lastTx = 0.0;
  private double lastTime = 0.0;

  public Shoot2Drive(
      Shooter shooter,
      CommandSwerveDrivetrain drivetrain,
      XboxController controller,
      XboxController drive) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.drive = drive;
    addRequirements(shooter, drivetrain);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    lastTime = Timer.getFPGATimestamp();
    lastTx = 0.0;

    double dist = shooter.getlong();
    boolean distValid = Double.isFinite(dist) && dist > 0.05 && dist < 10.0;

    if (distValid) {
      ShooterLookup.Point sp = ShooterLookup.sample(dist);
      lastTargetRpm = sp.rpm + kShotRpmOffset;
      lastTargetPitchRot = sp.pitchRot;
    }
  }

  @Override
  public void execute() {

    double xInput = -drive.getLeftY();
    double yInput = -drive.getLeftX();
    double omegaInput = drive.getRightX();

    double maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    double xMps = xInput ;
    double yMps = yInput ;
    double omegaRadPerSec = omegaInput ;

    Rotation2d heading = drivetrain.getTeleopHeading();

    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xMps,
        yMps,
        omegaRadPerSec,
        heading);

    drivetrain.setControl(driveRequest.withSpeeds(robotSpeeds));

    boolean manualHeld = controller.getYButton();
    double rx = drive.getRightX();

    if (manualHeld) {
      double manualYawOut = 0.0;

      if (Math.abs(rx) > kManualDeadband) {
        manualYawOut = MathUtil.clamp(rx * kManualScale, -1.0, 1.0);
      }

      shooter.setYawSpeed(manualYawOut);
      SmartDashboard.putBoolean("Auto/manualOverride", true);

      lastTime = Timer.getFPGATimestamp();
      lastTx = 0.0;

    } else {
      SmartDashboard.putBoolean("Auto/manualOverride", false);

      boolean hasTarget = shooter.hasLLTarget();
      double tx = shooter.getLLTx();
      double dist = shooter.getlong();

      boolean distValid = Double.isFinite(dist) && dist > 0.05 && dist < 10.0;

      if (hasTarget && distValid) {
        ShooterLookup.Point sp = ShooterLookup.sample(dist);
        lastTargetRpm = sp.rpm + kShotRpmOffset;
        lastTargetPitchRot = sp.pitchRot;
      }

      shooter.setPitchPosition(lastTargetPitchRot);
      shooter.setShooterRPM(lastTargetRpm);
 
      double yawCmd = 0.0;
      double now = Timer.getFPGATimestamp();
      double dt = now - lastTime;

      if (hasTarget) {
        if (Double.isFinite(tx) && Math.abs(tx) <= kMaxValidTxDeg) {
          double error = tx;

          if (Math.abs(error) > kYawTolDeg) {
            double errorRate = 0.0;

            if (dt > 1e-4) {
              errorRate = (error - lastTx) / dt;
            }

            yawCmd = error * kYawP + errorRate * kYawD;
            yawCmd = MathUtil.clamp(yawCmd, -kYawMaxOut, kYawMaxOut);
          }
        }
      }

      shooter.setYawSpeed(yawCmd);

      if (Double.isFinite(tx) && Math.abs(tx) <= kMaxValidTxDeg) {
        lastTx = tx;
      }

      lastTime = now;
    }

    boolean trigger = drive.getRightTriggerAxis() > kFireTrig;

    boolean rpmReady = Math.abs(shooter.getShooterRPM() - lastTargetRpm) <= kRpmTol;
    boolean timeReady = (Timer.getFPGATimestamp() - startTime) > kSpinupMinTime;

    boolean allowFeed = trigger;
    if (kGateByRPM) {
      allowFeed = trigger && rpmReady && timeReady;
    }

    if (allowFeed) {
      shooter.setTrainSpeed(kTrainDuty);
      shooter.setIntaketrainSpeed(kIntakeDuty);
    } else if (drive.getLeftTriggerAxis() > 0.4) {
      shooter.setTrainSpeed(-kTrainDuty);
      shooter.setIntaketrainSpeed(-kIntakeDuty);
    } else {
      shooter.setTrainSpeed(0.0);
      shooter.setIntaketrainSpeed(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(driveRequest.withSpeeds(new ChassisSpeeds()));
    shooter.setYawSpeed(0.0);
    shooter.setTrainSpeed(0.0);
    shooter.setIntaketrainSpeed(0.0);
    shooter.stopPitch();
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}