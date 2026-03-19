// package frc.robot.commands;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier; 
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;

// public class IntakeTest extends Command {
//   private final Intake intake;
//   private final Shooter m_Shooter;

//   private final DoubleSupplier up;     
//   private final DoubleSupplier down;   
//   private final DoubleSupplier isAPressed; 

//   public IntakeTest(Intake intake,Shooter m_Shooter, DoubleSupplier up, DoubleSupplier down, DoubleSupplier isAPressed) {
//     this.intake = intake;
//     this.m_Shooter = m_Shooter;
//     this.up = up;
//     this.down = down;
//     this.isAPressed = isAPressed;

//     addRequirements(intake,m_Shooter);
//   }

//   @Override
//   public void execute() {                           //   !!!!!!!!!記得灌程式 搖桿記得換!搖桿4號!!!!!!!!!!!!!!!!!
//     double rollingSpeed = isAPressed.getAsDouble(); // 右搖桿單獨控制 INTAKE長得很像水管的那個旋轉
//     intake.setRolling(rollingSpeed);                //自己用右搖桿上下調整旋轉方向

//     double anglespeed = down.getAsDouble();     // !!!按右邊的板機(寫R2的那個) 兩個傳輸加砲台一起轉 !!!
 
//     m_Shooter.setIntaketrainSpeed(anglespeed); //這行是 中間長得很像操場的傳輸的馬達(型號vortex)輸出 要反轉自己調整正負號
//     m_Shooter.setTrainSpeed(anglespeed);        //這行是 砲台下面傳輸 黑色輪子的那個 一樣自己用正負號調整旋轉方向
//     m_Shooter.setShooterSpeed(-anglespeed*0.8); //這行是 砲台前面兩顆馬達的輸出 一樣自己用正負號調整選轉方向

//     double Angle = up.getAsDouble();           // !!!左搖桿(左蘑菇頭) 調整INTAKE上下角度!!!
    
//       intake.setAngleSpeed(Angle*0.25);         //改這個可以控制速度 這個速度建議不要再快 用左搖桿的上下來調整旋轉方向就好

//   }

//   @Override
//   public void end(boolean interrupted) {
//     intake.setRolling(0);
// m_Shooter.setIntaketrainSpeed(0);
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }