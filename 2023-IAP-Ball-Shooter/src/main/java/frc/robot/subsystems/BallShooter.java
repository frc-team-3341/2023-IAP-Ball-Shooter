// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.feedForwardConsts;

public class BallShooter extends SubsystemBase {
  private BangBangController bang = new BangBangController();
  public final WPI_TalonSRX flywheel = new WPI_TalonSRX(Constants.BallHandlerPorts.flywheelPort);
  public final WPI_TalonSRX feedwheel = new WPI_TalonSRX(Constants.BallHandlerPorts.FeedwheelPort);
  public double ticks2RPM = 4096/10/60;
  private SimpleMotorFeedforward feedF = new SimpleMotorFeedforward(feedForwardConsts.kS, feedForwardConsts.kV, feedForwardConsts.kA);

  /** Creates a new BallShooter. */
  public BallShooter() {
   flywheel.configFactoryDefault();
   feedwheel.configFactoryDefault();
   flywheel.setInverted(true);
   resetEncoders();
   flywheel.setNeutralMode(NeutralMode.Coast);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  public double getRPM(){
    return flywheel.getSelectedSensorVelocity()/ticks2RPM;
  }

  public void resetEncoders(){
    flywheel.setSelectedSensorPosition(0);
  }

  public void setSpeed(double setPoint){
    flywheel.set(ControlMode.PercentOutput, bang.calculate(getRPM(), setPoint));
  }

  public void setSpeedSimple(double setPoint){
    flywheel.set(ControlMode.PercentOutput, setPoint);
  }

  public void setFeedSimple(double setPoint){
    feedwheel.set(ControlMode.PercentOutput, setPoint);
  }

  public void setSpeedff(double setPoint){
    flywheel.setVoltage(bang.calculate(getRPM(), setPoint)* 12.0 + 0.9 * feedF.calculate(setPoint));
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  public void periodic() {
    //setSpeedSimple(RobotContainer.getJoy().getY());
    if(RobotContainer.getJoy().getRawButtonPressed(6)){
      setSpeedSimple(0.1);
    }
    if(RobotContainer.getJoy().getRawButtonPressed(5)){
      setFeedSimple(0);
    }
    if(RobotContainer.getJoy().getRawButtonPressed(4)){
      setSpeedff(1000);
    }
    SmartDashboard.putNumber("RPM", getRPM());
  }

  // Called once the command ends or is interrupted.
}
