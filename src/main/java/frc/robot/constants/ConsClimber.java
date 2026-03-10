package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ConsClimber {

  public static final int RIGHT_CLIMBER_ID = 32;
  public static final int LEFT_CLIMBER_ID  = 31;

  public static final TalonFXConfiguration LEFT_CONFIG  = new TalonFXConfiguration();
  public static final TalonFXConfiguration RIGHT_CONFIG = new TalonFXConfiguration();

  static {
    LEFT_CONFIG.MotorOutput.NeutralMode  = NeutralModeValue.Brake;
    RIGHT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    LEFT_CONFIG.MotorOutput.Inverted  = InvertedValue.CounterClockwise_Positive;
    RIGHT_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs slot0 = new Slot0Configs()
        .withKP(1)
        .withKI(0)
        .withKD(0);

    LEFT_CONFIG.Slot0  = slot0;
    RIGHT_CONFIG.Slot0 = slot0;
  }

  private ConsClimber() {}
}