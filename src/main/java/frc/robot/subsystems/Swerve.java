package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.vision.AprilTagVision;
import frc.lib.HaNavX;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    private final Field2d m_field;

    // create swere drive Odomtry object
    public SwerveDriveOdometry swerveOdometry;
    
    // create swere module object
    public SwerveModule[] mSwerveMods;


    // create navex object
    private final HaNavX gyro;

    public Swerve() {

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        // give the path plat  hplaner all the metheds that he needs
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getSpeeds,
                this::driveRobotRelative,
                Constants.AutoConstants.pathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
        
        // get the navx from the robot and set his degris to 0
        gyro = new HaNavX(SPI.Port.kMXP);
        gyro.zeroYaw();

        // create all the modules
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants,false, true ),
                new SwerveModule(1, Constants.Swerve.Mod1.constants,false, true),
                new SwerveModule(2, Constants.Swerve.Mod2.constants, false, true),
                new SwerveModule(3, Constants.Swerve.Mod3.constants, false, true),
        };
        // set the swrve odometry
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    // drive methed
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // drive methed to the autonomus
    public void driveRobotRelative(ChassisSpeeds speeds) {

        // invers the rotaion 
        speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond * -1;

        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        DriverStation.reportWarning(Double.toString(speeds.omegaRadiansPerSecond), false);
        setModuleStates(states);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }


    public void resetOdometry(Pose2d pose) {
        setHeading(pose.getRotation().unaryMinus());
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getYawAngleDeg());
    }

    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }

    public void zeroHeading() {
        setHeading(Rotation2d.fromDegrees(0.0));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYawAngleDeg());
        // return gyro.getYaw();
    }

    public void resetModulesToAbsolute() {

        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }


    public double calculateDesinence(){
        Pose2d cerentPose2d = getPose();
        return Math.sqrt((Math.pow((16.793 - cerentPose2d.getX()), 2)) + (Math.pow(5.5478 - cerentPose2d.getY(), 2)) + (Math.pow(1.98, 2)));
    }

    public void crossWheels() {
        SwerveModuleState[] crossStates = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(115)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(225))
        };

        setModuleStates(crossStates);
    }

    public Command crossWheelsCommand() {
        return this.run(this::crossWheels);
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Command driveForwordInRobotRelativCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier rotationSup) {
        double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
        double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        return this.run(() -> drive(
                new Translation2d(-0.3, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                false,
                true));
    }

    @Override
    public void periodic() {  
        System.out.println(getPose() + "dectece: " + calculateDesinence());



        m_field.setRobotPose(getPose());



        // Update the odometry with the standard method
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        // Try to use AprilTag vision data to correct the odometry
        Optional<EstimatedRobotPose> visionPose = RobotContainer.aprilTagVision.getEstimatedGlobalPose(getPose());
    
        // If AprilTag vision provides a valid pose, use it to reset the odometry

        if (visionPose.isPresent()) {
            
            Pose2d estimatedPose = visionPose.get().estimatedPose.toPose2d();
            resetOdometry(estimatedPose); // Correct odometry with the vision-based pose
        }
    
       // Other existing periodic code...
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
    
}