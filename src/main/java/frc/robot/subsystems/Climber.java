package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConsClimber;

public class Climber extends SubsystemBase {

  private final TalonFX left = new TalonFX(ConsClimber.LEFT_CLIMBER_ID);
  private final TalonFX right = new TalonFX(ConsClimber.RIGHT_CLIMBER_ID);


  public Climber() {

    TalonFXConfiguration krakenConfigs = new TalonFXConfiguration();
    krakenConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    krakenConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    left.getConfigurator().apply(krakenConfigs);
    right.getConfigurator().apply(krakenConfigs);


  }

 public void setDoublespeed(double speed) {
    left.setControl(new DutyCycleOut(speed));
     right.setControl(new DutyCycleOut(-speed));
  }
  
}