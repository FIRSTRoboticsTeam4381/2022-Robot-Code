package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class FourBall extends SequentialCommandGroup {

        private String trajectoryJSON1 = "paths/4BallLeg1.wpilib.json";
        private Trajectory testTrajectory1 = new Trajectory();
        private String trajectoryJSON2 = "paths/4BallLeg2.wpilib.json";
        private Trajectory testTrajectory2 = new Trajectory();
        private String trajectoryJSON3 = "paths/4BallLeg3.wpilib.json";
        private Trajectory testTrajectory3 = new Trajectory();
        private String trajectoryJSON4 = "paths/4BallLeg4.wpilib.json";
        private Trajectory testTrajectory4 = new Trajectory();
        private String trajectoryJSON5 = "paths/4BallLeg5.wpilib.json";
        private Trajectory testTrajectory5 = new Trajectory();

        public FourBall(Swerve s_Swerve) {
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                .setKinematics(Constants.Swerve.swerveKinematics);

                try {
                        Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                        testTrajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                        Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                        testTrajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                        Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                        testTrajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                        Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                        testTrajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                        Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);
                        testTrajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON1, ex.getStackTrace());
                }

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
                                testTrajectory1,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                                testTrajectory2,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
                                testTrajectory3,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(
                                testTrajectory4,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand swerveControllerCommand5 = new SwerveControllerCommand(
                                testTrajectory5,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                addCommands(
                                new InstantCommand(() -> s_Swerve.resetOdometry(testTrajectory1.getInitialPose())),
                                swerveControllerCommand1, swerveControllerCommand2, swerveControllerCommand3, swerveControllerCommand4, swerveControllerCommand5);
        }
}