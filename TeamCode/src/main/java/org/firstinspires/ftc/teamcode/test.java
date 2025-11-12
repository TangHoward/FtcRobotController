package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="測試")
public class test extends LinearOpMode {

    public DcMotor mFR,mFL,mRR,mRL;
    public DcMotor mintake;
    @Override
    public void runOpMode() throws InterruptedException {
        mFR = hardwareMap.get(DcMotor.class,"mFR");
        mFL = hardwareMap.get(DcMotor.class,"mFL");
        mRR = hardwareMap.get(DcMotor.class,"mRR");
        mRL = hardwareMap.get(DcMotor.class,"mRL");

        mintake = hardwareMap.get(DcMotor.class,"mintake");

        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mintake.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        
        mFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.FORWARD);
        mRL.setDirection(DcMotorSimple.Direction.REVERSE);
        mRR.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while(opModeIsActive()){
            mFR.setPower((gamepad1.right_stick_y + gamepad1.left_stick_x + gamepad1.left_stick_x)*0.5);
            mFL.setPower((gamepad1.right_stick_y - gamepad1.left_stick_x - gamepad1.left_stick_x)*0.5);
            mRR.setPower((gamepad1.right_stick_y - gamepad1.left_stick_x + gamepad1.left_stick_x)*0.5);
            mRL.setPower((gamepad1.right_stick_y + gamepad1.left_stick_x - gamepad1.left_stick_x)*0.5);

            if(gamepad1.a){
                mintake.setPower(0.6);
            }else {
                mintake.setPower(0);
            }
        }
    }
}
