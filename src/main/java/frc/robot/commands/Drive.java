package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Drive extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier vX, vY, vOmega;

  private static final double kTransDeadband = 0.01;
  private static final double kRotDeadband   = 0.10;

  private final SwerveRequest.ApplyRobotSpeeds driveRequest =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public Drive(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier vOmega
  ) {
    this.drivetrain = drivetrain;
    this.vX = vX;
    this.vY = vY;
    this.vOmega = vOmega;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double x = MathUtil.applyDeadband(vX.getAsDouble(), kTransDeadband);
    double y = MathUtil.applyDeadband(vY.getAsDouble(), kTransDeadband);
    double omega = MathUtil.applyDeadband(vOmega.getAsDouble(), kRotDeadband);

    Rotation2d heading = drivetrain.getFieldHeading();

    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, heading);

    drivetrain.setControl(driveRequest.withSpeeds(robotSpeeds));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
