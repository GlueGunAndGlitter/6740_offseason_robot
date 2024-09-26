// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import javax.crypto.spec.GCMParameterSpec;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ChangeAngelShooter extends SubsystemBase {
  TalonFX rightAngelMotor = new TalonFX(Constants.AngelChangeConstants.RIGHT_ANGEL_MOTOR_ID);
  TalonFX leftAngelMotor = new TalonFX(Constants.AngelChangeConstants.LEFT_ANGEL_MOTOR_ID);
  
  PIDController rotatePID = new PIDController(0.2, 0, 0);
  /** Creates a new ChangeAngelShooter. */
  public ChangeAngelShooter() {
    leftAngelMotor.setInverted(true);
  }


  private void zeroEncoder(){
    leftAngelMotor.setPosition(0);
    rightAngelMotor.setPosition(0);
  }
  

  private void setRotaion(){
    double rotaion = (double) Robot.shooterAngel.getDouble(0) / 2;
    System.out.println(rotaion);
    if (rotatePID.calculate(leftAngelMotor.getPosition().getValue(),rotaion) > 0.5){
      leftAngelMotor.set(0.5);
      rightAngelMotor.set(0.5);
    }else{
      leftAngelMotor.set(rotatePID.calculate(leftAngelMotor.getPosition().getValue(),rotaion));
      rightAngelMotor.set(rotatePID.calculate(rightAngelMotor.getPosition().getValue(),rotaion));
    }

}

  private double getRotation(){
    return leftAngelMotor.getPosition().getValue();
  }

  private double getAngle(){
    return leftAngelMotor.getPosition().getValue() * 2;
  }
  private void stopMotor(){
    leftAngelMotor.stopMotor();
    rightAngelMotor.stopMotor();
  }

  private void setToZeroRotations(){
    
    if (rotatePID.calculate(leftAngelMotor.getPosition().getValue(),0) < -0.2 ){
      leftAngelMotor.set(-0.2);
      rightAngelMotor.set(-0.2);

    }else{
      leftAngelMotor.set(rotatePID.calculate(leftAngelMotor.getPosition().getValue(),0));
      rightAngelMotor.set(rotatePID.calculate(rightAngelMotor.getPosition().getValue(),0));
    }
  }


   public Command setToZeroRotationCommand(){
    return this.run(() -> setToZeroRotations());
   }
   public Command setRotationCommand(){
    return this.run(() -> setRotaion());
  }

  public Command stopMotorsCommand(){
    return this.run(() -> stopMotor());
  }

  public Command zeroEncodersCommand(){
    return this.run(() -> zeroEncoder());
  }
  @Override
  public void periodic() {
    // System.out.println(rotatePID.calculate(leftAngelMotor.getPosition().getValue(),0));
    // System.out.println("left: " + leftAngelMotor.getPosition() + "right: " + rightAngelMotor.getPosition());
  }
}
