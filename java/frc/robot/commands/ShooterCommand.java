package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterCommand extends Command {
  private ShooterSub shotinho;
    
  public ShooterCommand(ShooterSub shotinho) {
   this.shotinho = shotinho;

   addRequirements(shotinho);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (shotinho.HasTarget()){
      shotinho.shoot(
        shotinho.getDistanceToTag()
      );
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
