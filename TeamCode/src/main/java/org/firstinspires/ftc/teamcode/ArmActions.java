package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;

public class ArmActions {

    public DcMotor     ArmR        = null;
    public DcMotor     ArmL       = null;
    public DcMotor     SlideR     = null;
    public DcMotor     SlideL     = null;
    public CRServo     Intake     = null;
    public CRServo     WristR     = null;
    public CRServo     WristL     = null;

    public ArmActions(HardwareMap hardwareMap) {
        ArmR = hardwareMap.get(DcMotor.class, "ArmR");
        ArmL = hardwareMap.get(DcMotor.class, "ArmL");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        WristR = hardwareMap.get(CRServo.class, "WristR");
        WristL = hardwareMap.get(CRServo.class, "WristL");
        SlideR = hardwareMap.get(DcMotor.class, "SlideR");
        SlideL = hardwareMap.get(DcMotor.class, "SlideL");

        SlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    public Action chamberArm() {
        return new Action() {
            private boolean initialized = false;



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ArmR.setPower(0.7);
                    ArmL.setPower(-0.7);
                    initialized = true;
                }

                double pos = ArmR.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1000) {
                    return true;
                } else {
                    ArmR.setPower(0);
                    ArmL.setPower(0);
                    return false;
                }
            }
        };
    }
    public Action wallArm() {
        return new Action() {
            private boolean initialized = false;



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ArmR.setPower(-0.3);
                    ArmL.setPower(0.3);
                    initialized = true;
                }

                double pos = ArmR.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 400) {
                    return true;
                } else {
                    ArmR.setPower(0);
                    ArmL.setPower(0);
                    return false;
                }
            }
        };
    }

}
