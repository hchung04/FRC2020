package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PnumadicSystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private static DoubleSolenoid one = new DoubleSolenoid(0,1);

  public PnumadicSystem() {
  }


  public void expand(){
        one.set(Value.kForward); 
  }

  public void retract(){
    one.set(Value.kReverse);
}
  






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //let me commit
  }
}