package frc.robot.commands; 

import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.constants.ConsShooter; 
import frc.robot.subsystems.CommandSwerveDrivetrain; 
import frc.robot.subsystems.Shooter; 

public class ShootBack extends Command {  

  private final Shooter shooter;  
  private final CommandSwerveDrivetrain drivetrain; 

  private final double targetFieldDeg; 

  private final boolean invertGyroSign; 

  public ShootBack( 
      Shooter shooter, 
      CommandSwerveDrivetrain drivetrain, 
      double targetFieldDeg,
      boolean invertGyroSign 
  ) {
    this.shooter = shooter; 
    this.drivetrain = drivetrain;
    this.targetFieldDeg = targetFieldDeg; 
    this.invertGyroSign = invertGyroSign; 
    addRequirements(shooter); 
  }

  @Override
  public void initialize() { 
  }

  @Override
  public void execute() { 
    double robotDeg = drivetrain.getTeleopHeading().getDegrees(); 

    if (invertGyroSign) { 
      robotDeg = -robotDeg; 
    }

    double turretRelativeDeg = targetFieldDeg - robotDeg; 

    turretRelativeDeg = MathUtil.inputModulus( 
        turretRelativeDeg, 
        -180.0, 
        180.0 
    );

    double turretTargetRot = turretRelativeDeg / 360.0; 

    turretTargetRot = MathUtil.clamp( 
        turretTargetRot,
        ConsShooter.YAW_MIN_ROT, 
        ConsShooter.YAW_MAX_ROT 
    );

    shooter.setYawAbsRot(turretTargetRot);// rot

    // SmartDashboard.putNumber("ShootBack/robotDeg", robotDeg);
    // SmartDashboard.putNumber("ShootBack/targetFieldDeg", targetFieldDeg); 
    // SmartDashboard.putNumber("ShootBack/turretRelativeDeg", turretRelativeDeg); 
    // SmartDashboard.putNumber("ShootBack/turretTargetRot", turretTargetRot); 
  }

  @Override
  public void end(boolean interrupted) { 
    shooter.setYawSpeed(0.0); 
  }

  @Override
  public boolean isFinished() { 
    return false;
  }
}