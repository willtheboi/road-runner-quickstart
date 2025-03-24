package org.firstinspires.ftc.teamcode;


// RR-specific imports

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RSpecAutoRR", group = "Autonomous")
public class RSpecAutoRR extends LinearOpMode{

    @Override
    public void runOpMode() {

        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(9, -63.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ArmActions armActions = new ArmActions(hardwareMap);
        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-27, -53, Math.toRadians(235)), Math.toRadians(225));

                //.waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                //.setTangent(Math.toRadians(225))
                .strafeToConstantHeading(new Vector2d(-35, -61))
                .waitSeconds(2);

        waitForStart();

        if (isStopRequested()) return;

        Action trajectory_1;
        Action trajectory_2;

        trajectory_1 = tab1.build();
        trajectory_2 = tab2.build();


        Actions.runBlocking(
                new SequentialAction(
                        //arm up
                        //spline to chamber
                        //slide out
                        //wrist to chamber position
                        //drive forward to score
                        //spline to behind sample
                        //spline to obsv zone
                )


        );
    }
}