package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;



public class Armazenamento extends SubsystemBase {
  
  private SparkMax motorArmazenamento = new SparkMax(11, MotorType.kBrushed);
  private final DutyCycleEncoder spinCycle = new DutyCycleEncoder(4);

  private int cont = 0;

  public Armazenamento() {
    SparkMaxConfig configArmazenamento = new SparkMaxConfig();

    configArmazenamento.idleMode(SparkBaseConfig.IdleMode.kCoast);

    configArmazenamento.openLoopRampRate(0);

    motorArmazenamento.configure(
      configArmazenamento,
       ResetMode.kResetSafeParameters,
       PersistMode.kPersistParameters
    );
    spinCycle.setDutyCycleRange(0, 1);
  }
  public void CycleCont(){
    if (spinCycle.get() == 0.7){
      cont++;
    }
  }

  public int getCont(){
    return cont;
  }
  
  public void AutoSpin(){
    if (cont >= 16){
      motorArmazenamento.set(0);
    }
    else {
      motorArmazenamento.set(0.5);
    }
  }
  
  public void resetCont(){
    cont = 0;
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
