package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "IntoTheDeepAuton")
public class IntoTheDeepAuton extends LinearOpMode {

    Servo rightClaw;
    Servo leftClaw;

    Servo wrist;

    DcMotorEx LeftArm;
    DcMotorEx RightArm;

    public class Arm {

        public Action armSpecimen;

        public Arm(HardwareMap hardwareMap) {
            LeftArm = hardwareMap.get(DcMotorEx.class, "leftRobotArm");
            RightArm = hardwareMap.get(DcMotorEx.class, "rightRobotArm");

            LeftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        }
        public class ArmSpecimen implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    LeftArm.setPower(0.5);
                    RightArm.setPower(0.5);
                    initialized = true;
                }

                double pos = RightArm.getCurrentPosition();
                packet.put("liftPos", pos);

                if (pos <200) {
                    return true;
                }else {
                    LeftArm.setPower(0);
                    return false;
                }
            }
            public Action armSpecimen() {
                return new ArmSpecimen();
            }
        }
        }








    public class Claw {
        private Servo clawPivot;

        private Servo rightClaw;

        private Servo leftClaw;

        public Claw(HardwareMap hardwareMap) {
            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");
            clawPivot = hardwareMap.get(Servo.class, "wrist");

        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightClaw.setPosition(0.6);
                leftClaw.setPosition(0.35);

                return false;
            }

        }

        public Action openClaws() {
            return new OpenClaw();
        }

        public class ClawPivotGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                clawPivot.setPosition(0.55);

                return false;
            }

        }

        public Action clawPivotGround() {
            return new ClawPivotGround();
        }

        public class ClawPivotGrabSecondPixel implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawPivot.setPosition(0.6);

                return false;
            }
        }

        public Action clawPivotGrabSecondPixel() {
            return new ClawPivotGrabSecondPixel();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightClaw.setPosition(0.9);
                leftClaw.setPosition(0);

                return false;
            }

        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class ClawPivotUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                clawPivot.setPosition(0.3);

                return false;
            }

        }

        public Action clawPivotUp() {
            return new ClawPivotUp();
        }


        public void openClaw() {
            rightClaw.setPosition(0.6);
            leftClaw.setPosition(0.3);
        }
    }

        @Override
        public void runOpMode() throws InterruptedException {

            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");
            wrist = hardwareMap.get(Servo.class, "wrist");


            MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(60, -34, Math.toRadians(180)));
            Claw claw = new Claw(hardwareMap);
            Arm arm = new Arm(hardwareMap);
            TestAutonomous testAutonomous = new TestAutonomous();
            Action trajectoryAction1;

            trajectoryAction1 = mecanumDrive.actionBuilder(mecanumDrive.pose)
                    .splineToConstantHeading(new Vector2d(34,-10),Math.toRadians(180))
                    .stopAndAdd(claw.clawPivotUp())
                    .stopAndAdd(claw.openClaws())
                    .waitSeconds(2)
                    .splineToConstantHeading(new Vector2d(38,-45),Math.toRadians(180))
                    .stopAndAdd(claw.clawPivotGround())
                    .waitSeconds(1)
                    .stopAndAdd(claw.closeClaw())
                    .stopAndAdd(claw.clawPivotUp())
                    .waitSeconds(2)
                    .splineTo(new Vector2d(53, -50), Math.toRadians(310))
                    .stopAndAdd(claw.openClaws())
                    .waitSeconds(2)
                    .splineTo(new Vector2d(38, -55), Math.toRadians(180))
                    .stopAndAdd(claw.clawPivotGround())
                    .waitSeconds(1)
                    .stopAndAdd(claw.closeClaw())
                    .stopAndAdd(claw.clawPivotUp())
                    .waitSeconds(2)
                    .splineTo(new Vector2d(53, -50), Math.toRadians(310))
                    .stopAndAdd(claw.openClaws())
                    .waitSeconds(2)
                    .splineTo(new Vector2d(24, -55), Math.toRadians(270))
                    .stopAndAdd(claw.clawPivotGround())
                    .waitSeconds(1)
                    .stopAndAdd(claw.closeClaw())
                    .stopAndAdd(claw.clawPivotUp())
                    .waitSeconds(2)
                    .lineToY(-50)
                    .splineTo(new Vector2d(53, -50), Math.toRadians(130))
                    .stopAndAdd(claw.openClaws())
                    .build();

            wrist.setPosition(0);

            rightClaw.setPosition(0.9);
            leftClaw.setPosition(0);

            waitForStart();


            Actions.runBlocking(

                    new SequentialAction(


                    trajectoryAction1



                    )

            );

        }

}
