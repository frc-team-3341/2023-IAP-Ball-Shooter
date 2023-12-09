// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//randome line of code so that it pushes to github
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
  public double ticks2RPS = 4096/10;
  public double setPoint;
  private SimpleMotorFeedforward feedF = new SimpleMotorFeedforward(feedForwardConsts.kS, feedForwardConsts.kV, feedForwardConsts.kA);
  private boolean onOrOff;

  /** Creates a new BallShooter. */
  public BallShooter() {
   flywheel.configFactoryDefault();
   feedwheel.configFactoryDefault();
   flywheel.setInverted(false);
   resetEncoders();
   flywheel.setNeutralMode(NeutralMode.Coast);
   flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public double getRPS(){
    return flywheel.getSelectedSensorVelocity()/ticks2RPS*-1;
  }

  public void resetEncoders(){
    flywheel.setSelectedSensorPosition(0, 0, 10);
  }

  public void setSpeed(double setPoint){
    flywheel.set(ControlMode.PercentOutput, bang.calculate(getRPS(), setPoint));
  }

  public void setSpeedSimple(double setPoint){
    flywheel.set(ControlMode.PercentOutput, setPoint);
  }

  public void setFeedSimple(double setPoint){
    feedwheel.set(ControlMode.PercentOutput, setPoint);
  }

  public void setSpeedff(double setPoint){
    if(setPoint == 0){
      flywheel.set(ControlMode.PercentOutput, setPoint);
    }
    else{
      flywheel.setVoltage(bang.calculate(getRPS(), setPoint) + feedF.calculate(setPoint));
    }
    
    }
    
  

  public void setOnlyFF(double setPoint){
    flywheel.setVoltage(feedF.calculate(setPoint)/12.0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  
  public void periodic() {
    //setSpeedSimple(RobotContainer.getJoy().getY());
    if(RobotContainer.getJoy().getRawButtonPressed(1)){
      onOrOff =!onOrOff;
      if(onOrOff){
        setFeedSimple(0.9);
      }else{
        setFeedSimple(0);
      }
       }
    if(RobotContainer.getJoy().getRawButtonPressed(2)){
      setPoint = 0;

    }
    if(RobotContainer.getJoy().getRawButtonPressed(3)){
      
      setPoint = 97;
     
    }
    if(RobotContainer.getJoy().getRawButtonPressed(4)){
      setPoint = 90;
    }
    if(RobotContainer.getJoy().getRawButtonPressed(5)){
      setPoint = 85;
    }
    setSpeedff(setPoint);
    SmartDashboard.putNumber("RPS", getRPS());
    SmartDashboard.putBoolean("setpoint", bang.atSetpoint());
    SmartDashboard.putNumber("bang", bang.calculate(getRPS(), setPoint));
    SmartDashboard.putNumber("Feed Forward", 0.9 * feedF.calculate(setPoint)/12.0);
  }

  // Called once the command ends or is interrupted.
}
