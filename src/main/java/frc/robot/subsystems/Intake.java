// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

     CANSparkFlex rightMotor = new CANSparkFlex(Constants.ShooterConstants.NON_STATIC_MOTOR_PORT,
      MotorType.kBrushless);
     CANSparkFlex leftMotor = new CANSparkFlex(Constants.ShooterConstants.NON_STATIC_MOTOR_PORT,
      MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {}

  private void input(double rightSpeed, double leftSpeed){
    rightMotor.set(rightSpeed);
    leftMotor.set(leftSpeed);
  }

  private void output(double rightSpeed, double leftSpeed){
    rightMotor.set(-rightSpeed);
    leftMotor.set(-leftSpeed);
  }

  private void stopMotors(){
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public Command inputCommand(double rightSpeed, double leftSpeed){
    return this.run(() -> input(rightSpeed,leftSpeed));
  }

  public Command outputCommand(double rightSpeed, double leftSpeed){
    return this.run(() -> output(rightSpeed,leftSpeed));
  }

  public Command stopMotorsCommand(){
    return this.run(() -> stopMotors());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
