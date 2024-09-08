package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config// - for ftcdashboard
@TeleOp(name = "ArmPivot")
public class ArmPivot extends OpMode {
    PIDController pidController = new PIDController(kp,ki,kd);
    public static double kp = 0.005, kd = 0.0001, ki = 0, kf = 0.05, target =0;
    public static double ticksInInches = 720/180;// i tune this by extending my system to some place and dividing the read encoder ticks by the length(in your case degrees);
    public DcMotor LeftMotor;

    public DcMotor RightMotor;

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;

    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public DcMotorEx armExtender;

    private Servo rightClaw;
    private Servo leftClaw;


    private Servo clawPivot;


    public void closeClaw() {
        rightClaw.setPosition(0.9);
        leftClaw.setPosition(0);
    }
    public void openClaw() {
        rightClaw.setPosition(0.6);
        leftClaw.setPosition(0.3);
    }
    public void clawPivotBackdrop() {
        clawPivot.setPosition(0.1);
    }

    public void clawPositionTwo() {clawPivot.setPosition(0.5);}
    public void clawPositionOne() {clawPivot.setPosition(0.6);}
    public void clawPivotGround() {
        clawPivot.setPosition(0.7);
    }

    enum ArmStateFSM {

        Ready_To_Intake,

        Moving_To_Outtake,


        Ready_To_Outtake,

        Moving_To_Intake,

        Moving_To_Climb


    };

    ArmPivot.ArmStateFSM currentArmStates = ArmStateFSM.Ready_To_Intake;
    ElapsedTime timeSinceLastChange = new ElapsedTime();

    public void setArmState(ArmPivot.ArmStateFSM newState) {
        currentArmStates = newState;
        timeSinceLastChange.reset();
    }



    public void setTarget(double newTarget){
        this.target = newTarget;
    }

    @Override
    public void init() {
        LeftMotor = hardwareMap.get(DcMotor.class, "leftRobotArm");
        RightMotor = hardwareMap.get(DcMotor.class, "rightRobotArm");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        clawPivot = hardwareMap.get(Servo.class, "wrist");

        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        armExtender = hardwareMap.get(DcMotorEx.class, "armExtender");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    @Override
    public void loop() {
        pidController.setPID(kp, ki, kd);
        int armPos = (RightMotor.getCurrentPosition());
        double pid = pidController.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksInInches)) * kf;

        double power = pid + ff;

        RightMotor.setPower(power);
        LeftMotor.setPower(power);

        switch(currentArmStates) {
            case Ready_To_Intake:
                if(gamepad1.x){
                    setArmState(ArmStateFSM.Moving_To_Outtake);

                }
                break;


            case Moving_To_Outtake:
                closeClaw();
                if (timeSinceLastChange.milliseconds() > 1000) {
                    clawPivotBackdrop();

                    if (timeSinceLastChange.milliseconds()>1000) {
                        setTarget(585);

                        if (gamepad1.x) {
                            setArmState(ArmStateFSM.Ready_To_Outtake);
                        }
                    }
                }


                break;

            case Ready_To_Outtake:

                if (timeSinceLastChange.milliseconds()>1000) {
                    if (gamepad1.x) {
                        setArmState(ArmStateFSM.Moving_To_Intake);
                    } else if (gamepad1.left_stick_button) {
                        setTarget(650);
                    } else if (gamepad1.right_stick_button) {
                        setTarget(550);
                    }
                }

                break;

            case Moving_To_Intake:




                if (timeSinceLastChange.milliseconds() >300) {
                    setTarget(100);
                    if (gamepad1.x) {
                        setArmState(ArmStateFSM.Ready_To_Intake);
                    }
                }



                break;

            case Moving_To_Climb:

                clawPivotBackdrop();
                setTarget(500);



        }


        if(gamepad1.right_bumper) {
            armExtender.setPower(1);
        } else if(gamepad1.left_bumper) {
            armExtender.setPower(-1);
        } else{
            armExtender.setPower(0);
        }

        if (gamepad1.dpad_right) {
            openClaw();
        }
        if (gamepad1.dpad_left) {
            closeClaw();
        }

        if (gamepad1.dpad_down) {
            setArmState(ArmStateFSM.Moving_To_Intake);

        }

        if (gamepad1.dpad_up) {
            setArmState(ArmStateFSM.Moving_To_Climb);
        }

        if (gamepad1.a) {
            clawPivotGround();
        }

        if (gamepad1.b) {
            clawPositionOne();
        }

        if (gamepad1.y)  {
            clawPositionTwo();
        }

        if (gamepad1.options) {
            clawPivotBackdrop();
        }

        double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad2.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);


        telemetry.addData("rightArmPos: ", RightMotor.getCurrentPosition());
        telemetry.addData("currentState: ", currentArmStates);
        telemetry.update();

    }
}
