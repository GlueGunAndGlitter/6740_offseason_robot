// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ChangeAngelShooter extends SubsystemBase {
  TalonFX rightAngelMotor = new TalonFX(Constants.AngelChangeConstants.RIGHT_ANGEL_MOTOR_ID);
  TalonFX leftAngelMotor = new TalonFX(Constants.AngelChangeConstants.LEFT_ANGEL_MOTOR_ID);

  PIDController rotatePID = new PIDController(0.3, 0, 0);

  InterpolatingDoubleTreeMap estimatedAngle = new InterpolatingDoubleTreeMap();

  double angle = 0;

  /** Creates a new ChangeAngelShooter. */
  public ChangeAngelShooter() {
    putData();
    leftAngelMotor.setInverted(true);
  }

  private void setTargetAngle() {
    angle = estimatedAngle.get(RobotContainer.swerve.calculateDesinenceFromSpeaker());
  }

  private void zeroEncoder() {
    leftAngelMotor.setPosition(0);
    rightAngelMotor.setPosition(0);
  }

  /**
   * Sets the angle of the shooter to the given angle in degrees.
   * 
   * @param angle      the angle to set the shooter to in degrees
   * @param speedLimit the maximum speed to set the motor to
   */
  private void setAngle(double angle, double speedLimit) {
    double rotaion = (double) angle / 2;
    double speed = rotatePID.calculate(leftAngelMotor.getPosition().getValue(), rotaion);
    if (Math.abs(speed) > speedLimit) {
      int wight = (int) (Math.abs(speed) / speed);
      leftAngelMotor.set(speedLimit * wight);
      rightAngelMotor.set(speedLimit * wight);
    } else {
      leftAngelMotor.set(speed);
      rightAngelMotor.set(speed);
    }
  }

  private double getRotation() {
    return leftAngelMotor.getPosition().getValue();
  }

  private double getAngle() {
    return leftAngelMotor.getPosition().getValue() * 2;
  }

  private void stopMotor() {
    leftAngelMotor.stopMotor();
    rightAngelMotor.stopMotor();
  }

  private void putData() {
    estimatedAngle.put(5.34, 23.0);
    estimatedAngle.put(3.08, 35.0);
    estimatedAngle.put(4.23, 26.0);
    estimatedAngle.put(4.08, 26.0);
    estimatedAngle.put(2.4, 45.0);
    estimatedAngle.put(3.35, 32.0);
    estimatedAngle.put(4.4, 26.0);
    estimatedAngle.put(7.0, 21.7);

  }

  public Command setTargetAngaleCommand() {
    return this.runOnce(() -> setTargetAngle());
  }

  public Command setAngleFromShuffleboardCommand() {
    return this.run(() -> setAngle(Robot.shooterAngel.getDouble(0), 0.5));
  }

  public Command setToZeroAngleCommand() {
    return this.run(() -> setAngle(0, 0.2));
  }

  public Command setAngleCommand() {
    return this.run(() -> setAngle(angle, 0.5));
  }

  public Command setAngleToAmp(){
    return this.run(() -> setAngle(95, 0.5));
  }

  public Command stopMotorsCommand() {
    return this.run(() -> stopMotor());
  }

  public Command zeroEncodersCommand() {
    return this.runOnce(() -> zeroEncoder());
  }

  @Override
  public void periodic() {
    // System.out.println(estimatedAngle.get(RobotContainer.swerve.calculateDesinence()));
    // System.out.println(getAngle());

  }
}
