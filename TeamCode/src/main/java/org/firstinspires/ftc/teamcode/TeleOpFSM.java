package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpFSM")
public class TeleOpFSM extends OpMode {

    private Servo rightClaw;
    private Servo leftClaw;


    private Servo clawPivot;


    public void closeClaw() {
        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.5);
    }
    public void openClaw() {
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
    }
    public void clawPivotBackdrop() {
        clawPivot.setPosition(0);
    }
    public void clawPivotGround() {
        clawPivot.setPosition(0.5);
    }



    enum ArmState {

        Ready_To_Intake,

        Moving_To_Outtake,


        Ready_To_Outtake,

        Moving_To_Intake,

        Moving_To_Climb,

        Climbing

    };

    ArmState currentArmState = ArmState.Ready_To_Intake;
    ElapsedTime timeSinceLastChange = new ElapsedTime();

    public void setArmState(ArmState newState) {
        currentArmState = newState;
        timeSinceLastChange.reset();
    }
    @Override
    public void init() {
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");




    }
    @Override
    public void loop(){
        //ArmPivot armPivot = new ArmPivot(hardwareMap);
        ArmExtensionFSM armExtensionFSM = new ArmExtensionFSM(hardwareMap);


        switch(currentArmState) {
            case Ready_To_Intake:
                if(gamepad1.x){
                    setArmState(ArmState.Moving_To_Outtake);

                }
                break;


            case Moving_To_Outtake:
                closeClaw();
                if (timeSinceLastChange.milliseconds() > 400) {
                    //armPivot.setTarget(100);
                    armExtensionFSM.setTarget(100);
                    clawPivotBackdrop();
                    if (gamepad1.x){
                        setArmState(ArmState.Ready_To_Outtake);
                    }
                }


                break;

            case Ready_To_Outtake:

                if (timeSinceLastChange.milliseconds()>250) {
                    //armPivot.setTarget(400);
                    armExtensionFSM.setTarget(400);
                    if (gamepad1.x) {
                        setArmState(ArmState.Moving_To_Intake);
                    }
                }

                break;

            case Moving_To_Intake:
                clawPivotGround();
                openClaw();

                //armPivot.setTarget(100);
                armExtensionFSM.setTarget(100);

                if (gamepad1.x) {
                    setArmState(ArmState.Ready_To_Intake);
                }
                break;

            case Moving_To_Climb:

                clawPivotBackdrop();
                //armPivot.setTarget(180);
                armExtensionFSM.setTarget(400);
                break;

            case Climbing:
                armExtensionFSM.setTarget(350);
                break;

        }

        if (gamepad1.left_bumper) {
            openClaw();
        }
        if (gamepad1.right_bumper){
            closeClaw();
        }
        if (gamepad1.dpad_up) {setArmState(ArmState.Moving_To_Climb);}
        if (gamepad1.dpad_down) {setArmState(ArmState.Climbing);}
        //armPivot.update();
        armExtensionFSM.update();
    }
}
