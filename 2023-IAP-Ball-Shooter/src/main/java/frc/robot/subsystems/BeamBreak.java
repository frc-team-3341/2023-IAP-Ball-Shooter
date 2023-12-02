// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreak extends SubsystemBase {
  /** Creates a new Shooter. */
  DigitalInput beamBreaker = new DigitalInput(0);
  DigitalInput beamBreaker2 = new DigitalInput(9);
  Timer timer = new Timer();
  double seconds;

  public boolean getBeamBreaker(){
    return !beamBreaker.get();
  }

  public boolean getSecondBeamBreak(){
    return !beamBreaker2.get();
  }

  public void periodic() {
    SmartDashboard.putBoolean("Beam Break Sensor", getBeamBreaker());
    SmartDashboard.putBoolean("Second Beam Break Sensor", getSecondBeamBreak());
    if(getBeamBreaker()){
      timer.start();
    }
    if(getSecondBeamBreak()){
      timer.stop();
      if(timer.get() > 0){
        seconds = timer.get();
      }

      timer.reset();
    }

    SmartDashboard.putNumber("timer", timer.get());
    SmartDashboard.putNumber("seconds between sensors", seconds);
    // This method will be called once per scheduler run
  }
}
