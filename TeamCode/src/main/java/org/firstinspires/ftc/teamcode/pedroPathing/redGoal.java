package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;

import java.nio.file.Path;

@Autonomous(name = "redGoal")
public class redGoal extends OpMode {

    private Follower follower;


    private Flywheel flywheel = new Flywheel();

    private boolean shooting = false;


    private Timer pathTimer;

    private Timer opModeTimer;

    public enum PathState {
        //startPos_endPos

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        SHOOT_POS_END_POS,

    }


    public PathState pathState;

    private final Pose startPose = new Pose(122.19538670284939, 124.77526693045954, Math.toRadians(216));

    private final Pose shootPose = new Pose(90, 84.45590230664857, Math.toRadians(230));

    private final Pose endPose = new Pose(90, 130.9118046132971, Math.toRadians(270));

    private PathChain driveStartshoot;

    private PathChain driveShootend;

    double amount = 22.0;
    double lastTime;


    public void buildPaths() {
        driveStartshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootend = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartshoot, true);
                setPathState(PathState.SHOOT_PRELOAD);  // reset timer & make new state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    //TODO: FLYWHEEL SHOOT
                    if (!shooting) {
                        flywheel.fireShots(3);
                        shooting = true;
                    } else if (shooting && !flywheel.isBusy()) {
                        shooting = false;
                        telemetry.addLine("Finished Shooting");
                        follower.followPath(driveShootend, true);
                        setPathState(PathState.SHOOT_POS_END_POS);
                    }
                }
                break;
            //transition to next state
            case SHOOT_POS_END_POS:

            default:
                telemetry.addLine("No State Selected");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        telemetry.addLine("Path took " + pathTimer.getElapsedTimeSeconds());
        pathTimer.resetTimer();;

        shooting = false;
    }

    @Override
    public void init() {
        pathState = pathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        flywheel.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {

        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        follower.update();
        flywheel.update(dt);
        statePathUpdate();

        telemetry.addData("Command Velocity:", flywheel.getCommandedVelocity());
        telemetry.addData("Target Velocity:", flywheel.getTargetVelocity());
        //telemetry.addData("Top Motor Velocity:", flywheel.getVelocity(0));
        //telemetry.addData("Side Motor Velocity:", flywheel.getVelocity(1));
        telemetry.addData("Flywheel State: ", flywheel.getState());
        telemetry.addData("Path State: ", pathState.toString());
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Path Time: ", pathTimer.getElapsedTimeSeconds());;

    }
}

