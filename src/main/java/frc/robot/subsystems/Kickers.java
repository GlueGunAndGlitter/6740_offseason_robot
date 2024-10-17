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

public class Kickers extends SubsystemBase {
       CANSparkFlex motorKicker = new CANSparkFlex(Constants.KickersConstans.KICKER_MOTOR_ID,
      MotorType.kBrushless);
    
  /** Creates a new Kickers. */
  public Kickers() {
    motorKicker.setInverted(true);

    motorKicker.setSmartCurrentLimit(40);
    motorKicker.setSecondaryCurrentLimit(50);
  }


  private void inputKicker(){
    motorKicker.set(Robot.KickerMotorShafelbordSpeed.getDouble(0));
  }

  public void outputKicker(){
    motorKicker.set(-Robot.KickerMotorShafelbordSpeed.getDouble(0));
  }

  private void stopMotors(){
    motorKicker.stopMotor();
  }

  public Command inputKickerCommand() {
    return this.run(() -> inputKicker());
  }

  public Command outputKickerCommand() {
    return this.run(() -> outputKicker());
  }

  public Command stopMotorsCommand(){
    return this.run(() -> stopMotors());  
  }


  @Override
  public void periodic() {
    // System.out.println(motorKicker.getMotorTemperature());
    // This method will be called once per scheduler run
    
  }
}
