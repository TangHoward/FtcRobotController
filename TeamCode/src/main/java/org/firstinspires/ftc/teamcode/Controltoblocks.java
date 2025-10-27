package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Controltoblocks extends BlocksOpModeCompanion {
    private static DcMotor mFL,mFR,mRR,mRL;

    private static GoBildaPinpointDriver pinPoint;
    @ExportToBlocks(
            heading = "初始化"
    )
    public static void Init(){
            mFL = hardwareMap.get(DcMotor.class,"mFL");
            mFR = hardwareMap.get(DcMotor.class,"mFR");
            mRL = hardwareMap.get(DcMotor.class,"mRL");
            mRR = hardwareMap.get(DcMotor.class,"mRR");

            pinPoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinPoint");
    }

    @ExportToBlocks(
            heading = "操控移動",
            parameterLabels = {"速度"},
            parameterDefaultValues = {"0.7"}
    )
    public static void Move(double maxspeed){
        pinPoint.update();
        /*double headingRad = Math.toRadians(heading);
        double robotVx = targetVxGlobal * Math.cos(headingRad) - targetVyGlobal * Math.sin(headingRad);
        double robotVy = targetVxGlobal * Math.sin(headingRad) + targetVyGlobal * Math.cos(headingRad);*/
        double robotVx = gamepad1.left_stick_x * Math.cos(pinPoint.getHeading(AngleUnit.RADIANS))
                            - gamepad1.left_stick_y * Math.sin(pinPoint.getHeading(AngleUnit.RADIANS));

        double robotVy = -gamepad1.left_stick_x * Math.sin(pinPoint.getHeading(AngleUnit.RADIANS))
                       - gamepad1.left_stick_y * Math.cos(pinPoint.getHeading(AngleUnit.RADIANS));

        /*double powerFL = robotVy + robotVx + targetOmega;
        double powerFR = robotVy - robotVx - targetOmega;
        double powerRL = robotVy - robotVx + targetOmega;
        double powerRR = robotVy + robotVx - targetOmega;*/
        double powerFL = (robotVy +  robotVx + gamepad1.right_stick_x);
        double powerFR = (robotVy -  robotVx - gamepad1.right_stick_x);
        double powerRL = (robotVy -  robotVx + gamepad1.right_stick_x);
        double powerRR = (robotVy +  robotVx - gamepad1.right_stick_x);

        double maxRawPower = Math.max(Math.max(Math.abs(powerFL),Math.abs(powerFR)),Math.max(Math.abs(powerRL),Math.abs(powerRR)));

        double scaleFactor = 1;

        if(maxRawPower > 1) {
            scaleFactor = maxRawPower;
        }else if(maxRawPower > maxspeed){
            scaleFactor = maxRawPower / maxspeed;
        }
        powerFL /= scaleFactor;
        powerFR /= scaleFactor;
        powerRL /= scaleFactor;
        powerRR /= scaleFactor;

        mFL.setPower(powerFL);
        mFR.setPower(powerFR);
        mRL.setPower(powerRL);
        mRR.setPower(powerRR);


    }
}
