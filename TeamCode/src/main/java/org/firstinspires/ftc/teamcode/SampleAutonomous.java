package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "testAutonomous")
public class SampleAutonomous extends LinearOpMode {
    public class Lift {
        private DcMotorEx armPivotLeft;
        private DcMotorEx armPivotRight;

        private DcMotorEx armExtension;

        public Lift(HardwareMap hardwareMap) {
            armPivotRight = hardwareMap.get(DcMotorEx.class, "armPivotRight");
            armPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            armPivotLeft = hardwareMap.get(DcMotorEx.class, "armPivotLeft");
            armPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");
            armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        public class PivotBackdrop implements Action {
            private boolean initialized = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armPivotRight.setPower(0.5);
                    armPivotLeft.setPower(0.5);
                    initialized = true;
                }

                double pos = armPivotRight.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos<780) {
                    return true;
                } else {
                    armPivotLeft.setPower(0);
                    armPivotRight.setPower(0);
                    return false;
                }

            }
        }public Action pivotBackdrop(){
            return new PivotBackdrop();
        }
        public class PivotGround implements Action {
            private boolean initialized = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armPivotRight.setPower(-0.5);
                    armPivotLeft.setPower(-0.5);
                    initialized = true;
                }

                double pos = armPivotRight.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos>0) {
                    return true;
                } else {
                    armPivotLeft.setPower(0);
                    armPivotRight.setPower(0);
                    return false;
                }
            }

        }public Action pivotGround(){
            return new PivotGround();
        }



    }
    public class Claw {
        private Servo rightClaw;
        private Servo leftClaw;

        private Servo clawPivot;

        public Claw (HardwareMap hardwareMap) {
            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");
            clawPivot = hardwareMap.get(Servo.class, "clawPivot");

        }

        public class CloseRightClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rightClaw.setPosition(0.5);
                return false;
            }
        }
        public Action closeRightClaw() {
            return new CloseRightClaw();
        }

        public class CloseLeftClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                leftClaw.setPosition(0.5);
                return false;
            }
        }
        public Action closeLeftClaw() {
            return new CloseLeftClaw();
        }

        public class OpenRightClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rightClaw.setPosition(1);
                return false;
            }
        }
        public Action openRightClaw() {
            return new OpenRightClaw();
        }

        public class OpenLeftClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                leftClaw.setPosition(1);
                return false;
            }
        }
        public Action openLeftClaw() {
            return new OpenLeftClaw();
        }

        public class ClawResting implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawPivot.setPosition(0.5);
                return false;
            }
        }
        public Action clawResting() {
            return new ClawResting();
        }

        public class ClawBackdrop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawPivot.setPosition(1);
                return false;
            }
        }
        public Action clawBackrop() {
            return new ClawBackdrop();
        }


    }
    @Override
    public void runOpMode() {
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        AprilTagDrive aprilTagDrive = new AprilTagDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        armExtension ArmExtension = new armExtension(hardwareMap);

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryActionCloseOut;

        trajectoryAction1 = aprilTagDrive.actionBuilder(aprilTagDrive.pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .build();

        trajectoryAction2 = aprilTagDrive.actionBuilder(aprilTagDrive.pose)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .lineToX(47.5)
                .waitSeconds(3)
                .build();

        trajectoryActionCloseOut = aprilTagDrive.actionBuilder(aprilTagDrive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();


        Actions.runBlocking(claw.closeRightClaw());
        Actions.runBlocking(claw.closeLeftClaw());


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                trajectoryAction1,
                                claw.clawResting(),
                                claw.openRightClaw()
                        ),
                        new ParallelAction(
                                trajectoryAction2,
                                lift.pivotBackdrop(),
                                ArmExtension.ArmUp(),
                                claw.clawBackrop()
                        ),

                        trajectoryActionCloseOut

                )
        );







    }
}
