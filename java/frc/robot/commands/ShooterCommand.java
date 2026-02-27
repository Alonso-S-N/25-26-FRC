package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Armazenamento;
import frc.robot.subsystems.ShooterSub;

public class ShooterCommand extends Command {
  private ShooterSub shotinho;
  private Armazenamento armazenamento;
  public ShooterCommand(ShooterSub shotinho, Armazenamento armazenamento) {
   this.shotinho = shotinho;
   this.armazenamento = armazenamento;

   addRequirements(shotinho,armazenamento);
  }

  @Override
  public void initialize() {
    armazenamento.resetCont();
  }

  @Override
  public void execute() {
    if (shotinho.HasTarget() && shotinho.ValidShootID(shotinho.getTagID())){
      shotinho.shoot(
        shotinho.getDistanceToCenter()
      );
      armazenamento.AutoSpin();
    } else if (armazenamento.getCont() >= 16){ 
      shotinho.StopShooter();
    } else {
      shotinho.StopShooter();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shotinho.StopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
