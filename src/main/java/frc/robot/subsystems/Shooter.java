// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  CANSparkFlex frontUpMotor = new CANSparkFlex(Constants.ShooterConstants.FRONT_UP_MOTOR_ID,
  MotorType.kBrushless);
  CANSparkFlex frontDownMotor = new CANSparkFlex(Constants.ShooterConstants.FONT_DOWN_MOTOR_ID,
  MotorType.kBrushless);
  CANSparkFlex backDownMotor = new CANSparkFlex(Constants.ShooterConstants.BACK_DOWN_MOTOR_ID,
  MotorType.kBrushless);


  
  /** Creates a new Shooter. */
  
  public Shooter() {
    

    frontDownMotor.setSmartCurrentLimit(35);
    frontDownMotor.setSecondaryCurrentLimit(50);

    frontUpMotor.setSmartCurrentLimit(40);
    frontUpMotor.setSecondaryCurrentLimit(50);

    backDownMotor.setSmartCurrentLimit(40);
    backDownMotor.setSecondaryCurrentLimit(50);
  

    frontDownMotor.setInverted(false);
    frontUpMotor.setInverted(true);
    backDownMotor.setInverted(false);


  }
   

  private void shootUp(){

    frontUpMotor.set(Robot.ShooterfrontUpMotorSpeed.getDouble(0));
    frontDownMotor.set(Robot.ShooterfrontDownMotorSpeed.getDouble(0));
    backDownMotor.set(-Robot.ShooterBackDownMotorSpeed.getDouble(0));
  }

  private void insert(){
    frontUpMotor.set(-Robot.ShooterfrontUpMotorSpeed.getDouble(0));
    frontDownMotor.set(-Robot.ShooterfrontDownMotorSpeed.getDouble(0));
    backDownMotor.set(Robot.ShooterBackDownMotorSpeed.getDouble(0));
  }

  private void floorInput(){
    frontUpMotor.set(0);
    frontDownMotor.set(Robot.ShooterfrontDownMotorSpeed.getDouble(0)-0.5);
    backDownMotor.set(Robot.ShooterBackDownMotorSpeed.getDouble(0)-0.2);
  }

  private void ampShot(){
    frontUpMotor.set(-Robot.ShooterfrontUpMotorSpeed.getDouble(0));
    frontDownMotor.set(0);
    backDownMotor.set(-Robot.ShooterBackDownMotorSpeed.getDouble(0)-0.85);
  }

  private double getRPM(){
    double RPM = frontDownMotor.getEncoder().getVelocity();
    return RPM;
  }

  public Command shootUpCommand(){
    return this.run(() -> shootUp());
  }

  public Command insertCommand(){
    return this.run(() -> insert());
  }

  public Command floorInputCommand(){
    return this.run(() -> floorInput());
  }

  public Command ampshotCommand(){
    return this.run(() -> ampShot());
  }

  
  
  //public Command intakeShooterCommand(){
   // return this.run(() -> zeroMotors(1));
 // }
  
  private void stopMotors(){
    frontUpMotor.stopMotor();
    frontDownMotor.stopMotor();
    backDownMotor.stopMotor();
    
  }
  
  
  public Command stopMotorsCommand(){
    return this.run(() -> stopMotors());
  }
  @Override
  public void periodic() {
    // System.out.println(getRPM());
    // This method will be called once per scheduler run
    
  }
}
