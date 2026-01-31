package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;



public class Armazenamento extends SubsystemBase {
  
  private SparkMax motorArmazenamento = new SparkMax(19, MotorType.kBrushless);

  public Armazenamento() {
    SparkMaxConfig configArmazenamento = new SparkMaxConfig();

    configArmazenamento.idleMode(SparkBaseConfig.IdleMode.kCoast);

    configArmazenamento.openLoopRampRate(0);

    motorArmazenamento.configure(
      configArmazenamento,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters
    );

  }

  public void setMotorArmazenamento(double Speed){
    motorArmazenamento.set(Speed);
  }

  public void StopMotorArmazenamento(){
    motorArmazenamento.set(0);
  }

  @Override
  public void periodic() {
  }
}
