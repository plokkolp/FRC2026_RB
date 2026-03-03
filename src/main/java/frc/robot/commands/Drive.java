package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

public class Drive extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier vX, vY, vOmega;

  private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private static final double kDeadband = 0.08;

  private static final double kMaxOmegaRadPerSec = 1 * Math.PI;

  public Drive(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier vOmega) {
    this.drivetrain = drivetrain;
    this.vX = vX;
    this.vY = vY;
    this.vOmega = vOmega;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    double xInput = vX.getAsDouble();
    double yInput = vY.getAsDouble();
    double omegaInput = -vOmega.getAsDouble();

    double maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    double xMps = xInput * maxSpeedMps;
    double yMps = yInput * maxSpeedMps;
    double omegaRadPerSec = omegaInput * kMaxOmegaRadPerSec;

    Rotation2d heading = drivetrain.getTeleopHeading();

    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xMps,
        yMps,
        omegaRadPerSec,
        heading);

    drivetrain.setControl(
        driveRequest.withSpeeds(robotSpeeds));

    // SmartDashboard.putNumber("Drive/x_mps", xMps);
    // SmartDashboard.putNumber("Drive/y_mps", yMps);
    // SmartDashboard.putNumber("Drive/omega_radps", omegaRadPerSec);
    // SmartDashboard.putNumber("Drive/maxSpeed", maxSpeedMps);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}