

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends /*LoggedRobot */ TimedRobot{
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
   // Logger.recordMetadata("ProjectName", "SwerveBinga"); 
    //Logger.recordMetadata("RuntimeType", RobotBase.getRuntimeType().toString());
  
    //if (RobotBase.isSimulation()) {
     // Logger.addDataReceiver(new WPILOGWriter("logs"));
     // Logger.addDataReceiver(new NT4Publisher());
    //} else {
     // Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
     // Logger.addDataReceiver(new NT4Publisher());
    //}
    //Logger.start();
   // try {
    // Logger.start();
 //} catch (Exception e) {
   //  e.printStackTrace();
 //} 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    Command auto = m_robotContainer.getAutonomousCommand();
    if (auto != null) {
        auto.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
