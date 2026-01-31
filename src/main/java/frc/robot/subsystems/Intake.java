package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final TalonFX haveRolling = new TalonFX(11);
  private final TalonFX trainRolling = new TalonFX(12);
  private final TalonFX haveangle = new TalonFX(13);

  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  public Intake() {

    TalonFXConfiguration krakenConfigs = new TalonFXConfiguration();
    krakenConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    krakenConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    haveRolling.getConfigurator().apply(krakenConfigs);
    
    krakenConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    trainRolling.getConfigurator().apply(krakenConfigs);

    krakenConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    haveangle.getConfigurator().apply(krakenConfigs);
  }

  public void haveSpeed(double speed) {
    haveRolling.setControl(new DutyCycleOut(speed));
  }

  public void trainSpeed(double speed) {
    trainRolling.setControl(new DutyCycleOut(speed));
  }

  public double getAnglePosition() {
    return haveangle.getPosition().getValueAsDouble();
  }
  public void setAnglePosition(double rotations) {
    haveangle.setControl(m_mmReq.withPosition(rotations));
  }
  public void setAnglespeed(double speed) {
    haveangle.setControl(new DutyCycleOut(speed));
  }
   @Override
    public void periodic() {
      SmartDashboard.putNumber("havePosition", getAnglePosition());
    }
  }