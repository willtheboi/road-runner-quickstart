package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.MecanumDrive;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Autonomous(name = "Pathway_Test", group = "Autonomous")
public class Pathway_Test extends LinearOpMode{
    private DcMotor ArmR     = null;
    private DcMotor ArmL     = null;
    private CRServo Intake   = null;
    private CRServo WristR   = null;
    private CRServo WristL   = null;
    private DcMotor SlideR   = null;
    private DcMotor SlideL   = null;
    @Override
    public void runOpMode() {
        ArmR    = hardwareMap.get(DcMotor.class, "ArmR");
        ArmL    = hardwareMap.get(DcMotor.class, "ArmR");
        Intake      = hardwareMap.get(CRServo.class, "Intake");
        WristR      = hardwareMap.get(CRServo.class, "WristR");
        WristL      = hardwareMap.get(CRServo.class, "WristL");
        SlideR      = hardwareMap.get(DcMotor.class, "SlideR");
        SlideL      = hardwareMap.get(DcMotor.class, "SlideL");

        ArmL.setDirection(DcMotor.Direction.FORWARD);
        ArmR.setDirection(DcMotor.Direction.REVERSE);
        SlideL.setDirection(DcMotor.Direction.REVERSE);
        SlideR.setDirection(DcMotor.Direction.FORWARD);

        ArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(9, -63.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-50)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(50,-20,Math.toRadians(-90)), Math.toRadians(90));
                //.waitSeconds(3);
        waitForStart();
        ArmR.setPower(0.5);
        ArmL.setPower(0.5);
        sleep(500);
        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );
    }
}