package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "TestAutonomous")
public class TestAutonomous extends LinearOpMode {



    public class Claw {
        private Servo clawPivot;

        private Servo rightClaw;

        private Servo leftClaw;

        public Claw (HardwareMap hardwareMap) {
            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");
            clawPivot = hardwareMap.get(Servo.class, "wrist");

        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rightClaw.setPosition(0.6);
                leftClaw.setPosition(0.35);

                return false;
            }

        }
        public Action openClaw() {
            return new OpenClaw();
        }
        public class ClawPivotGround implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){

                clawPivot.setPosition(0.65);

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

        public class CloseClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rightClaw.setPosition(0.9);
                leftClaw.setPosition(0);

                return false;
            }

        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class ClawPivotUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){

                clawPivot.setPosition(0.3);

                return false;
            }

        }
        public Action clawPivotUp() {
            return new ClawPivotUp();
        }



    }

    @Override
    public void runOpMode() throws InterruptedException {

        Servo rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        Servo leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        Claw claw = new Claw(hardwareMap);

        rightClaw.setPosition(0.9);
        leftClaw.setPosition(0);





        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));

        Action trajectoryAction1;
        Action trajectoryAction2;

        trajectoryAction1 = mecanumDrive.actionBuilder(mecanumDrive.pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .build();

        trajectoryAction2 = mecanumDrive.actionBuilder(mecanumDrive.pose)
                .setTangent(Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .splineTo(new Vector2d(44,28), Math.toRadians(-90))
                .build();

        waitForStart();

        Actions.runBlocking(

                new SequentialAction(

                        trajectoryAction1,

                        claw.clawPivotGround(),

                        new SleepAction(3),

                        claw.openClaw(),

                        new SleepAction(2),

                        claw.clawPivotGrabSecondPixel(),

                        new SleepAction(2),

                        claw.closeClaw(),

                        new SleepAction(2),

                        claw.clawPivotUp(),

                        new SleepAction(2),

                        trajectoryAction2,

                        claw.openClaw(),

                        new SleepAction(3),


                        new SleepAction(2)



                        )





        );


    }
}
