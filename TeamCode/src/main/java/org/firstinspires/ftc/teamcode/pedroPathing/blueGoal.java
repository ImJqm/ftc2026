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

@Autonomous(name = "blueGoal")
public class blueGoal extends OpMode {

    private Follower follower;



    private Timer pathTimer;

    private Timer opModeTimer;

    public enum PathState {
        //startPos_endPos

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD

    }


    public PathState pathState;

    private final Pose startPose = new Pose(21.947473484552496, 124.77526693045954, Math.toRadians(324));

    private final Pose shootPose = new Pose(54.436906377204885, 84.45590230664857, Math.toRadians(310));

    private PathChain driveStartshoot;

    public void buildPaths() {
        driveStartshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
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
                    telemetry.addLine("flywheel shoot");
                }
                break;
                //transition to next state
            default:
                telemetry.addLine("No State Selected");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        telemetry.addLine("Path took " + pathTimer.getElapsedTimeSeconds());
        pathTimer.resetTimer();;
    }



    @Override
    public void init() {
        pathState = pathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

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
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State: ", pathState.toString());
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Path Time: ", pathTimer.getElapsedTimeSeconds());;

    }
}

