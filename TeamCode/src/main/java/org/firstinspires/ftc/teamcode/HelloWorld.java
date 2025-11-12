package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;


public class HelloWorld extends BlocksOpModeCompanion {
    @ExportToBlocks(
            parameterLabels = {"要顯示的字串"},
            parameterDefaultValues = {"Helloworld"}
    )
    public void Helloworld(String word){
        telemetry.addData("顯示的字",word);
        telemetry.update();
    }

}
