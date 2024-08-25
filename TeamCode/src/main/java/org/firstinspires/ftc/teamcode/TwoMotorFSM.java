package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "twomotorfsm")
public class TwoMotorFSM extends OpMode {
    private DcMotorEx rightFront;

    private DcMotorEx leftFront;

    public void state0() {
        rightFront.setPower(0);
        leftFront.setPower(0);

    }

    public void state1() {
        rightFront.setPower(0.5);
    }

    public void state2() {
        leftFront.setPower(0.5);
    }

    public void state3() {
        rightFront.setPower(0.5);
        leftFront.setPower(0.5);
    }

    enum MotorState {
        State0,
        State1,
        State2,
        State3
    }

    MotorState currentMotorState = MotorState.State0;

    ElapsedTime timeSinceLastChange = new ElapsedTime();

    public void setMotorState(MotorState newState) {
        currentMotorState = newState;
        timeSinceLastChange.reset();
    }

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

    }
    @Override
    public void loop() {
        switch(currentMotorState) {
            case State0:
                state0();
                if (gamepad1.x) {
                    setMotorState(MotorState.State1);
                }
                break;
            case State1:
                rightFront.setPower(0);
                leftFront.setPower(0);
                if (timeSinceLastChange.milliseconds()>1000) {
                    state1();
                    if (gamepad1.x) {
                        setMotorState(MotorState.State2);
                    }
                }
                break;
            case State2:
                rightFront.setPower(0);
                leftFront.setPower(0);
                if (timeSinceLastChange.milliseconds()>1000) {
                    state2();
                    if (gamepad1.x) {
                        setMotorState(MotorState.State3);
                    }
                }
                break;
            case State3:
                rightFront.setPower(0);
                leftFront.setPower(0);
                if (timeSinceLastChange.milliseconds()>1000) {
                    state3();
                    if (gamepad1.x) {
                        setMotorState(MotorState.State0);
                    }
                }
                break;
        }
        if (gamepad1.right_bumper) {
            setMotorState(MotorState.State0);
        }
        if (gamepad1.left_bumper) {
            setMotorState(MotorState.State3);
        }

        telemetry.addData("currentMotorState", currentMotorState);
        telemetry.update();
    }

}
