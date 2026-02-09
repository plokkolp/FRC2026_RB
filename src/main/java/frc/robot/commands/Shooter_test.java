package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class Shooter_test extends InstantCommand {

  private final Shooter shooter;
  private DoubleSupplier targetDeg;
  private final BooleanSupplier KevinDurant;
  
    public Shooter_test(Shooter shooter,DoubleSupplier targetDeg, BooleanSupplier KevinDurant) {
      this.shooter = shooter;
      this.targetDeg = targetDeg;
      this.KevinDurant = KevinDurant;
      addRequirements(shooter);
    }
  
    @Override
    public void initialize() {
  }
    @Override
  public void execute() {
   double KD = targetDeg.getAsDouble();
   boolean MJ = KevinDurant.getAsBoolean();

   shooter.setYawSpeed(KD*0.1);
   shooter.setShooterRPM(3000);
   if(MJ){
      shooter.setTrainSpeed(-0.5);
    } else{
       shooter.setTrainSpeed(0);

    }

  }

   @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
