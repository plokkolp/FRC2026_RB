// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;

// public class Shoot0227 extends Command {
//   private final Shooter shooter;
//   private final XboxController controller; 


//   public Shoot0227(Shooter shooter, XboxController controller) {
//     this.shooter = shooter;
//     this.controller = controller;

//     addRequirements(shooter);
//   }
//   @Override
//   public void initialize() {}
  
//   @Override
//   public void execute() {


//     shooter.setShooterSpeed(-0.4); 
    
//   if(controller.getRightTriggerAxis() > 0.9){
//     shooter.setTrainSpeed(-0.6); 
//         shooter.setIntaketrainSpeed(-0.2);

//   }else{
//     shooter.setTrainSpeed(0);
//         shooter.setIntaketrainSpeed(-0);
 
//   }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     shooter.setShooterSpeed(0); 
//   }
  
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }