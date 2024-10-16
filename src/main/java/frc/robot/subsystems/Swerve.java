package frc.robot.subsystems;

import frc.robot.SwerveModule;
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
                                rotation),Constants.Swerve.robotCenterTranslation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // drive methed to the autonomus
    public void driveRobotRelative(ChassisSpeeds speeds) {

        // invers the rotaion 
        speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond * -1;

        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds, Constants.Swerve.robotCenterTranslation);
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
        // setHeading(pose.getRotation().unaryMinus());
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
    }

    public void resetModulesToAbsolute() {

        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    private static void setAlineWithSpeakerAngel(){
        Pose2d robotPose = RobotContainer.swerve.getPose();
        double m;
        double angle;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
         m = (robotPose.getY() - 5.5478)/(robotPose.getX() + 0.0381);
         angle= (180 -Math.toDegrees(Math.atan(m)));
        }else{
         m =(robotPose.getY() - 5.5478)/(robotPose.getX() - 16.5793);
        //  System.out.println("m: " + m + " and: " + (180 + Math.toDegrees(Math.atan(m))));
        angle = (180 + Math.toDegrees(Math.atan(m)));
        }

        RobotContainer.alineWithSpeakerAngel = angle;
     }

     public Command setAlineWithSpeakerAngelCommand(){
        return this.runOnce(()-> setAlineWithSpeakerAngel());
     }

    public double calculateDesinenceFromSpeaker(){
        Pose2d cerentPose2d = getPose();
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            return Math.sqrt((Math.pow((-0.0381 - cerentPose2d.getX()), 2)) + (Math.pow(5.5478 - cerentPose2d.getY(), 2)) + (Math.pow(1.98, 2)));
        }
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

    @Override
    public void periodic() {  
        // System.out.println(RobotContainer.angle);

        Pose2d robotPose = getPose();
        // System.out.println( Math.sqrt((Math.pow((16.793 - robotPose.getX()), 2)) + (Math.pow(5.5478 - robotPose.getY(), 2))));
  
        m_field.setRobotPose(getPose());



        // Update the odometry with the standard method
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        // Try to use AprilTag vision data to correct the odometry
        Optional<Pose2d> visionPose = RobotContainer.aprilTagVision.getEstimatedGlobalPose(getPose());
    
        // If AprilTag vision provides a valid pose, use it to reset the odometry

        // System.out.println(RobotContainer.alineWithSpeakerAngel +  "    " +  getHeading().getDegrees());

        if (visionPose.isPresent()) {
            
            Pose2d estimatedPose = visionPose.get();
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