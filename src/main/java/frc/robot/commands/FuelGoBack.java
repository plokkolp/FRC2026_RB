package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class FUELGOBACK extends Command {

  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;
  private final XboxController driver;

  private static final double kEnableRangeDeg = 90.0;

  private static final double kCenterRot = -0.147705078125; 
  private static final double kRotPerDeg = -0.01607;        

  private static final double kMinRot = -1.759521484375 - 0.147949218750 + 0.25; 
  private static final double kMaxRot =  1.13330078125  - 0.147949218750 + 0.25; 

  private static final double kP = 0.8;
  private static final double kMaxOut = 0.25;
  private static final double kTolRot = 0.01;

  public FUELGOBACK(Shooter shooter,CommandSwerveDrivetrain drivetrain,XboxController driver) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.driver = driver;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    double poseDeg = drivetrain.getPose().getRotation().getDegrees();
    poseDeg = MathUtil.inputModulus(poseDeg, -180.0, 180.0);

    boolean enabled = Math.abs(poseDeg) <= kEnableRangeDeg;

    double curRot = shooter.getYawMotorPositionRot();
    double desiredDeg = 0.0;
    double targetRot = curRot;
    double errRot = 0.0;
    double out = 0.0;

    if (enabled) {
      desiredDeg = -poseDeg;
      desiredDeg = MathUtil.inputModulus(desiredDeg, -180.0, 180.0);

      targetRot = kCenterRot + desiredDeg * kRotPerDeg;
      targetRot = MathUtil.clamp(targetRot, kMinRot, kMaxRot);

      errRot = targetRot - curRot;

      if (Math.abs(errRot) < kTolRot) {
        out = 0.0;
      } else {
        out = MathUtil.clamp(errRot * kP, -kMaxOut, kMaxOut);
      }

      shooter.setYawSpeed(out);

      if (driver.getRightTriggerAxis() > 0.3) {
        shooter.setPitchPosition(-2.45);

        if (shooter.getPitchPositionRot() < -2.1) {
          shooter.setShooterSpeed(0.9);
          shooter.setIntaketrainSpeed(-0.7);
          shooter.setTrainSpeed(-0.7);
        } else {
          shooter.setShooterSpeed(0.0);
          shooter.setIntaketrainSpeed(0.0);
          shooter.setTrainSpeed(0.0);
        }
      } else {
        shooter.setPitchPosition(-0.23);
        shooter.setShooterSpeed(0.0);
        shooter.setIntaketrainSpeed(0.0);
        shooter.setTrainSpeed(0.0);
      }

    } else {
      shooter.setYawSpeed(0.0);

    }

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0.0);
  }
}