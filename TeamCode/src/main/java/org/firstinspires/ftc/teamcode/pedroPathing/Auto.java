package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configablestuff;

@Autonomous(name = "紅方自動", group = "RED")
public class Auto extends OpMode {
    //變數設定
    private TelemetryManager telemetryM;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = -1;

    private final Pose startingPose = new Pose(91, 8, Math.toRadians(0));
    private final Pose shootPose = new Pose(84, 15, Math.toRadians(65.55604522));
    private final Pose humanElementsPose = new Pose(126, 15, Math.toRadians(0));
    private final Pose firstElement = new Pose(126, 15, Math.toRadians(0));
    private final Pose firstElementControlPoint = new Pose(84, 38.3, Math.toRadians(38));
    private final Pose gate = new Pose(120, 72, Math.toRadians(90));
    private final Pose gateControlPoint = new Pose(77.51154313487243, 76.98663426488457, Math.toRadians(90));

    private Path auto;
    private PathChain pickHumanElement, shootHumanElement, pick1stElement, shoot1stElement, goToGate;


    public void buildPath(){
        auto = new Path(new BezierLine(startingPose, shootPose));
        auto.setLinearHeadingInterpolation(startingPose.getHeading(),shootPose.getHeading());

        pickHumanElement = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,humanElementsPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),humanElementsPose.getHeading(),0.8)
                .build();

        shootHumanElement = follower.pathBuilder()
                .addPath(new BezierLine(humanElementsPose,shootPose))
                .setLinearHeadingInterpolation(humanElementsPose.getHeading(),shootPose.getHeading(),0.3)
                .build();

        pick1stElement = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,firstElementControlPoint,firstElement))
                .setLinearHeadingInterpolation(shootPose.getHeading(),firstElement.getHeading(),0.5)
                .build();

        shoot1stElement = follower.pathBuilder()
                .addPath(new BezierLine(firstElement,shootPose))
                .setLinearHeadingInterpolation(firstElement.getHeading(),shootPose.getHeading())
                .build();

        goToGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,gateControlPoint,gate))
                .setLinearHeadingInterpolation(shootPose.getHeading(),gate.getHeading(),0)
                .build();

    }

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(auto);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(pickHumanElement,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(shootHumanElement,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    actionTimer.resetTimer();
                    setPathState(30);
                }
                break;
            case 30:
                if (actionTimer.getElapsedTimeSeconds() >= 1.5){
                    //轉馬達
                    follower.followPath(pick1stElement,true);
                    setPathState(4);
                } else {
                    telemetry.addData("動作狀態","正在射擊 (還有時間: %.2f)",1.5-actionTimer.getElapsedTimeSeconds());
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(shoot1stElement,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(goToGate,true);
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        Configablestuff.botPose = follower.getPose();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        buildPath();
        follower.setStartingPose(startingPose);

        telemetryM.debug("A1. 起點到射擊點", auto);
        telemetryM.debug("A2. 拾取人類元素", pickHumanElement);
        telemetryM.debug("A3. 射擊人類元素", shootHumanElement);
        telemetryM.debug("A4. 拾取第一個元素", pick1stElement);
        telemetryM.debug("A5. 射擊第一個元素", shoot1stElement);
        telemetryM.debug("A6. 駛向大門", goToGate);

        telemetryM.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
