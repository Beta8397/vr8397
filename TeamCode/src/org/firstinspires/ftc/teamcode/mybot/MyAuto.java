package org.firstinspires.ftc.teamcode.mybot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.QuinticSpline2D;


public abstract class MyAuto extends LinearOpMode {

    protected MyBot bot;

    public void setBot(MyBot bot){
        this.bot = bot;
    }

    public void driveTo(MotionProfile mProfile, double targetX, double targetY,
                        double targetHeadingDegrees, double tolerance){
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        Pose startPose = bot.getPose();

        while (opModeIsActive()){
            bot.updateOdometry();
            Pose pose = bot.getPose();
            VectorF fromStart = new VectorF((float)(pose.x - startPose.x), (float)(pose.y - startPose.y));
            VectorF toTarget = new VectorF((float)(targetX - pose.x), (float)(targetY - pose.y));
            double d1 = fromStart.magnitude();
            double d2 = toTarget.magnitude();

            if (d2 < tolerance){
                break;
            }

            VectorF dir = toTarget.multiplied(1.0f/(float)d2);

            double v1 = Math.sqrt(mProfile.vMin * mProfile.vMin + 2.0 * mProfile.accel * d1);
            double v2 = Math.sqrt(mProfile.vMin * mProfile.vMin + 2.0 * mProfile.accel * d2);
            double v = Math.min(v1, v2);
            v = Math.min(v, mProfile.vMax);

            double vX = v * dir.get(0);
            double vY = v * dir.get(1);

            double sin = Math.sin(pose.h);
            double cos = Math.cos(pose.h);

            double vXR = vX * sin - vY * cos;
            double vYR = vX * cos + vY * sin;

            double vA = 2.0 * AngleUnit.normalizeRadians(targetHeadingRadians - pose.h);

            bot.setDriveSpeed(vXR, vYR, vA);
        }

        bot.setDrivePower(0, 0, 0);
    }

    public void turnTo(double targetHeadingDegrees, double vaMaxDegrees, double coeff, double toleranceDegrees){
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        double vaMaxRadians = Math.toRadians(vaMaxDegrees);
        double toleranceRadians = Math.toRadians(toleranceDegrees);

        while (opModeIsActive()){
            bot.updateOdometry();
            Pose pose = bot.getPose();
            Pose velocity = bot.getVelocity();
            double offset = AngleUnit.normalizeRadians(targetHeadingRadians - pose.h);

            if (Math.abs(offset) < toleranceRadians && Math.abs(velocity.h) < 0.5){
                break;
            }

            double va = coeff * offset;
            if (Math.abs(va) > vaMaxRadians){
                va = vaMaxRadians * Math.signum(va);
            }

            bot.setDriveSpeed(0, 0, va);
        }

        bot.setDrivePower(0, 0, 0);
    }


    /**
     * Drive quintic spline from current position and heading to target position and heading.
     * @param mProf     Min and max speed, acceleration
     * @param endX      target X coordinate
     * @param endY      target Y coordinate
     * @param endHeadingDegrees     Target final TRAVEL DIRECTION
     * @param tolerance             target final coordinate tolerance, inches
     * @param reverse               If true, drive in reverse
     */
    public void splineTo(MotionProfile mProf, double endX, double endY,
                         double endHeadingDegrees, double tolerance, boolean reverse){
        bot.updateOdometry();
        double startHeadingRadians = bot.getPose().h;
        if (reverse){
            startHeadingRadians = AngleUnit.normalizeRadians(startHeadingRadians + Math.PI);
        }
        double endHeadingRadians = Math.toRadians(endHeadingDegrees);
        VectorF endVec = new VectorF((float)endX, (float)endY);
        QuinticSpline2D spline = new QuinticSpline2D(bot.getPose().x, bot.getPose().y,
                endX, endY, startHeadingRadians, endHeadingRadians);
        double totalLength = spline.getLength();
        VectorF prevTargVec = spline.p(0);
        double s = 0;
        double d0 = 0;
        double absDistToEnd = 0;
        double headingError = 0;
        VectorF vel;
        VectorF velRobot;
        double va = 0;

        while (opModeIsActive()){
            bot.updateOdometry();
            VectorF poseVec = new VectorF((float)bot.getPose().x, (float)bot.getPose().y);

            // Find closest point on spline to current pose
            s = spline.findClosestS(poseVec.get(0), poseVec.get(1), s);

            // Distance (crow flies) between bot pose and end point
            absDistToEnd = poseVec.subtracted(endVec).magnitude();

            // If at or beyond end of spline or within tolerance, break
            if (s >= 0.9999999 || absDistToEnd < tolerance) break;

            // Where bot should be now along spline
            VectorF currTargVec = spline.p(s);

            // Update distance traveled (d0) along spline so far, and find
            // remaining distance along spline (d1) to end
            VectorF deltaTargVec = currTargVec.subtracted(prevTargVec);
            d0 += deltaTargVec.magnitude();
            if (d0 > totalLength) d0 = totalLength;
            double d1 = totalLength - d0;

            // targErr is vector from current bot pose to where bot should be on spline
            VectorF targErr = currTargVec.subtracted(poseVec);

            // travelDir is unit vector giving current direction of travel along spline
            VectorF travelDir = spline.d1(s);
            travelDir = travelDir.multiplied(1.0f/travelDir.magnitude());

            // Compute nominal travel speed, v
            double v0 = Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            double v1 = Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            double v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            // Compute travel velocity, including nominal velocity plus correction
            vel = travelDir.multiplied((float)v).added(targErr.multiplied(8.0f));

            // Convert vel from field coordinates to robot coordinates
            velRobot = fieldToBot(vel, bot.getPose().h);

            // Compute target heading at current location on spline, and heading rate of change
            double targetHeading = spline.getHeading(s);
            if (reverse) {
                targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
            }
            double targetHeadingRate = spline.getHeadingRateOfChange(s, v);

            // Compute current heading error
            headingError = AngleUnit.normalizeRadians(targetHeading - bot.getPose().h);

            // Compute heading speed, including nominal rate of change plus correction
            va = targetHeadingRate + 4.0 * headingError;

            // Set robot speed
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);

            prevTargVec = currTargVec;
        }

        /*
         * We have either reached the end of the spline or reached end point within tolerance.
         * In case tolerance not reached, allow 1 second period of adjustment
         */

        ElapsedTime et = new ElapsedTime();

        double targetHeading = endHeadingRadians;
        if (reverse) {
            targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
        }

        while (opModeIsActive() && et.seconds() < 1){
            bot.updateOdometry();
            VectorF vecToEnd = endVec.subtracted(new VectorF((float)bot.getPose().x, (float)bot.getPose().y));
            headingError = AngleUnit.normalizeRadians(targetHeading - bot.getPose().h);
            if (vecToEnd.magnitude() < tolerance && Math.abs(headingError) < Math.toRadians(3)) break;
            vel = vecToEnd.multiplied(8);
            velRobot = fieldToBot(vel, bot.getPose().h);
            va = 4.0 * headingError;
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);
        }

        bot.setDriveSpeed(0,0,0);
    }


    /**
     * Drive quintic spline from current position and heading to target position and heading.
     * @param mProf     Min and max speed, acceleration
     * @param endX      target X coordinate
     * @param endY      target Y coordinate
     * @param endHeadingDegrees     Target final TRAVEL DIRECTION
     * @param tolerance             target final coordinate tolerance, inches
     * @param reverse               If true, drive in reverse
     * @param alpha     Higher alpha -> less turning at the beginning, more at the middle of path
     * @param beta      Higher beta -> less turning at the end, more at the middle of path
     */
    public void splineTo(MotionProfile mProf, double endX, double endY, double endHeadingDegrees,
                         double tolerance, boolean reverse, double alpha, double beta){
        bot.updateOdometry();
        double startHeadingRadians = bot.getPose().h;
        if (reverse){
            startHeadingRadians = AngleUnit.normalizeRadians(startHeadingRadians + Math.PI);
        }
        double endHeadingRadians = Math.toRadians(endHeadingDegrees);
        VectorF endVec = new VectorF((float)endX, (float)endY);
        QuinticSpline2D spline = new QuinticSpline2D(bot.getPose().x, bot.getPose().y,
                endX, endY, startHeadingRadians, endHeadingRadians, alpha, beta);
        double totalLength = spline.getLength();
        VectorF prevTargVec = spline.p(0);
        double s = 0;
        double d0 = 0;
        double absDistToEnd = 0;
        double headingError = 0;
        VectorF vel;
        VectorF velRobot;
        double va = 0;

        while (opModeIsActive()){
            bot.updateOdometry();
            VectorF poseVec = new VectorF((float)bot.getPose().x, (float)bot.getPose().y);

            // Find closest point on spline to current pose
            s = spline.findClosestS(poseVec.get(0), poseVec.get(1), s);

            // Distance (crow flies) between bot pose and end point
            absDistToEnd = poseVec.subtracted(endVec).magnitude();

            // If at or beyond end of spline or within tolerance, break
            if (s >= 0.9999999 || absDistToEnd < tolerance) break;

            // Where bot should be now along spline
            VectorF currTargVec = spline.p(s);

            // Update distance traveled (d0) along spline so far, and find
            // remaining distance along spline (d1) to end
            VectorF deltaTargVec = currTargVec.subtracted(prevTargVec);
            d0 += deltaTargVec.magnitude();
            if (d0 > totalLength) d0 = totalLength;
            double d1 = totalLength - d0;

            // targErr is vector from current bot pose to where bot should be on spline
            VectorF targErr = currTargVec.subtracted(poseVec);

            // travelDir is unit vector giving current direction of travel along spline
            VectorF travelDir = spline.d1(s);
            travelDir = travelDir.multiplied(1.0f/travelDir.magnitude());

            // Compute nominal travel speed, v
            double v0 = Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            double v1 = Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            double v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            // Compute travel velocity, including nominal velocity plus correction
            vel = travelDir.multiplied((float)v).added(targErr.multiplied(8.0f));

            // Convert vel from field coordinates to robot coordinates
            velRobot = fieldToBot(vel, bot.getPose().h);

            // Compute target heading at current location on spline, and heading rate of change
            double targetHeading = spline.getHeading(s);
            if (reverse) {
                targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
            }
            double targetHeadingRate = spline.getHeadingRateOfChange(s, v);

            // Compute current heading error
            headingError = AngleUnit.normalizeRadians(targetHeading - bot.getPose().h);

            // Compute heading speed, including nominal rate of change plus correction
            va = targetHeadingRate + 4.0 * headingError;

            // Set robot speed
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);

            prevTargVec = currTargVec;
        }

        /*
         * We have either reached the end of the spline or reached end point within tolerance.
         * In case tolerance not reached, allow 1 second period of adjustment
         */

        ElapsedTime et = new ElapsedTime();

        double targetHeading = endHeadingRadians;
        if (reverse) {
            targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
        }

        while (opModeIsActive() && et.seconds() < 1){
            bot.updateOdometry();
            VectorF vecToEnd = endVec.subtracted(new VectorF((float)bot.getPose().x, (float)bot.getPose().y));
            headingError = AngleUnit.normalizeRadians(targetHeading - bot.getPose().h);
            if (vecToEnd.magnitude() < tolerance && Math.abs(headingError) < Math.toRadians(3)) break;
            vel = vecToEnd.multiplied(8);
            velRobot = fieldToBot(vel, bot.getPose().h);
            va = 4.0 * headingError;
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);
        }

        bot.setDriveSpeed(0,0,0);
    }




    protected VectorF fieldToBot(VectorF vField, double heading) {
        float sinTheta = (float) Math.sin(heading);
        float cosTheta = (float) Math.cos(heading);
        return new VectorF(vField.get(0) * sinTheta - vField.get(1) * cosTheta, vField.get(0) * cosTheta + vField.get(1) * sinTheta);
    }

    protected VectorF botToField(VectorF vBot, double heading){
        float sinTheta = (float) Math.sin(heading);
        float cosTheta = (float) Math.cos(heading);
        return new VectorF(vBot.get(0) * sinTheta + vBot.get(1) * cosTheta, -vBot.get(0) * cosTheta + vBot.get(1) * sinTheta);
    }






}
