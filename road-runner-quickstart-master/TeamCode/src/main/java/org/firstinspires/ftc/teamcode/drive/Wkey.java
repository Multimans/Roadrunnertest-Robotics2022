package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
//actually ignore this for now
@Config
public class Wkey extends SampleMecanumDrive{
    @Override
    public void runOpMode(){
        Wkey drive = new Wkey(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        /*
         *
         * travel_distance("forward", (2*610));
            travel_distance("forward", (1*610));
            travel_distance("left", (1*610));
            rotate_degrees("counterclockwise", (90));
            rotate_degrees("counterclockwise", (180));

            travel_distance("forward", (2*610));
            travel_distance("forward", (3*610));
            rotate_degrees("counterclockwise", (180));

            travel_distance("forward", (1*610));
             rotate_degrees("counterclockwise", (180));
            travel_distance("forward", (1*610));
            rotate_degrees("counterclockwise", (180));

            travel_distance("forawrd", (2*610));
            travel_distance("left", (1*610));
            travel_distance("right", (1*610));
            travel_distance("forward", (3*610));
         */

        drive.setPoseEstimate(startPose);

        Trajectory begging = drive.trajectoryBuilder(startPose)
                .forward(4)
                .build();

        Trajectory next1 = drive.trajectoryBuilder(begging.end())
                .forward(2)
                .strafeLeft(2)
                .build();

        Trajectory next2 = drive.trajectoryBuilder(next1.end())
                .strafeLeft(2)
                .build();

        Trajectory next3 = drive.trajectoryBuilder(next2.end(), Math.toRadians(270))
                .forward(4)
                .build();

        Trajectory next4 = drive.trajectoryBuilder(next3.end())
                .forward(6)
                .build();

        Trajectory next5 = drive.trajectoryBuilder(next4.end(), Math.toRadians(180))
                .forward(2)
                .build();

        Trajectory next6 = drive.trajectoryBuilder(next5.end(), Math.toRadians(180))
                .forward(2)
                .build();


        Trajectory next7 = drive.trajectoryBuilder(next6.end(), Math.toRadians(180))
                .forward(4)
                .build();

        Trajectory next8 = drive.trajectoryBuilder(next7.end())
                .strafeLeft(2)
                .build();

        Trajectory next9 = drive.trajectoryBuilder(next8.end())
                .strafeRight(2)
                .build();

        Trajectory finish = drive.trajectoryBuilder(next9.end())
                .forward(6)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(begging);
        drive.followTrajectory(next1);
        drive.followTrajectory(next2);
        drive.turn(Math.toRadians(90));
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(next3);
        drive.followTrajectory(next4);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(next5);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(next6);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(next7);
        drive.followTrajectory(next8);
        drive.followTrajectory(next9);
        drive.followTrajectory(finish);



    }
}
























}
