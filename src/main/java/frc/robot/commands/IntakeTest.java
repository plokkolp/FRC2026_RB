package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier; 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeTest extends Command {
  private final Intake intake;
  private final Shooter m_Shooter;

  private final DoubleSupplier up;     
  private final DoubleSupplier down;   
  private final DoubleSupplier isAPressed; 

  public IntakeTest(Intake intake,Shooter m_Shooter, DoubleSupplier up, DoubleSupplier down, DoubleSupplier isAPressed) {
    this.intake = intake;
    this.m_Shooter = m_Shooter;
    this.up = up;
    this.down = down;
    this.isAPressed = isAPressed;

    addRequirements(intake,m_Shooter);
  }

  @Override
  public void execute() {
    double rollingSpeed = isAPressed.getAsDouble();
    intake.setRolling(rollingSpeed);

    double anglespeed = down.getAsDouble();
    m_Shooter.setIntaketrainSpeed(-anglespeed); 
    m_Shooter.setTrainSpeed(anglespeed);
    m_Shooter.setShooterSpeed(-anglespeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setRolling(0);
m_Shooter.setIntaketrainSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}