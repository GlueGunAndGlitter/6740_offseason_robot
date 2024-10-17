// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LineWithSpeaker extends PIDCommand {
  /** Creates a new LineWithSpeaker. */

  public LineWithSpeaker(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationSup) {
        super(
          // The controller that the command will use
          new PIDController(Constants.LineWithSpeakerConstants.KP, 0, Constants.LineWithSpeakerConstants.KD),
          // This should return the measurement
          () -> Math.IEEEremainder(RobotContainer.swerve.getHeading().getDegrees(), 360),
          // This should return the setpoint (can also be a constant)
          () -> Math.IEEEremainder(RobotContainer.alineWithSpeakerAngel,360),
          // This uses the output
          output -> {

            double megerment = Math.IEEEremainder(RobotContainer.swerve.getHeading().getDegrees(), 360);
            double setpoint = Math.IEEEremainder(RobotContainer.alineWithSpeakerAngel,360);
            // Use the output here
            double err = Math.IEEEremainder(RobotContainer.swerve.getHeading().getDegrees(), 360) - Math.IEEEremainder(RobotContainer.alineWithSpeakerAngel,360);
            double alternativeOutput = 2;
            if (Math.abs(err) > 180){
              alternativeOutput = (360 + err *  (Math.abs(Math.IEEEremainder(RobotContainer.alineWithSpeakerAngel,360))/ Math.IEEEremainder(RobotContainer.alineWithSpeakerAngel,360))) * Constants.LineWithSpeakerConstants.KP;
              alternativeOutput = alternativeOutput * -(Math.abs(Math.IEEEremainder(RobotContainer.alineWithSpeakerAngel,360))/ Math.IEEEremainder(RobotContainer.alineWithSpeakerAngel,360));
            }

            output = Math.min(Math.abs(alternativeOutput), Math.abs(output));
            if ((megerment < setpoint && (megerment > 0 || (megerment < 0 && setpoint < 0))) || (megerment > 0 && setpoint < 0)) {
              output = -output;
            }
            // apply deadband
            double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
            double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);

            RobotContainer.swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            -output * Constants.Swerve.maxAngularVelocity,
            false,
            true);
          }
          );
          // Use addRequirements() here to declare subsystem dependencies.
          // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
