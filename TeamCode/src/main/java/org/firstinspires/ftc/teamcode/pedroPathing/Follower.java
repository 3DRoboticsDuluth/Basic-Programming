//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.DriveVectorScaler;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.Constants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.FilteredPIDFController;
import com.pedropathing.util.KalmanFilter;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Follower {
    private HardwareMap hardwareMap;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;
    private DriveVectorScaler driveVectorScaler;
    public PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Pose closestPose;
    private Path currentPath;
    private PathChain currentPathChain;
    private int BEZIER_CURVE_SEARCH_LIMIT;
    private int AVERAGED_VELOCITY_SAMPLE_NUMBER;
    private int chainIndex;
    private long[] pathStartTimes;
    private boolean followingPathChain;
    private boolean holdingPosition;
    private boolean isBusy;
    private boolean isTurning;
    private boolean reachedParametricPathEnd;
    private boolean holdPositionAtEnd;
    private boolean teleopDrive;
    private double globalMaxPower = (double)1.0F;
    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    private double holdPointTranslationalScaling;
    private double holdPointHeadingScaling;
    public double driveError;
    public double headingError;
    private long reachedParametricPathEndTime;
    private double[] drivePowers;
    private double[] teleopDriveValues;
    private ArrayList<Vector> velocities = new ArrayList();
    private ArrayList<Vector> accelerations = new ArrayList();
    private Vector averageVelocity;
    private Vector averagePreviousVelocity;
    private Vector averageAcceleration;
    private Vector secondaryTranslationalIntegralVector;
    private Vector translationalIntegralVector;
    private Vector teleopDriveVector;
    private Vector teleopHeadingVector;
    public Vector driveVector;
    public Vector headingVector;
    public Vector translationalVector;
    public Vector centripetalVector;
    public Vector correctiveVector;
    private double centripetalScaling;
    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryTranslationalIntegral;
    private PIDFController translationalPIDF;
    private PIDFController translationalIntegral;
    private PIDFController secondaryHeadingPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;
    private FilteredPIDFController drivePIDF;
    private KalmanFilter driveKalmanFilter;
    private double[] driveErrors;
    private double rawDriveError;
    private double previousRawDriveError;
    private double turnHeadingErrorThreshold;
    public static boolean drawOnDashboard = true;
    public static boolean useTranslational = true;
    public static boolean useCentripetal = true;
    public static boolean useHeading = true;
    public static boolean useDrive = true;
    private boolean cached = false;
    private VoltageSensor voltageSensor;
    public double voltage = (double)0.0F;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private boolean logDebug = true;
    private ElapsedTime zeroVelocityDetectedTimer;

    public Follower(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
        this.hardwareMap = hardwareMap;
        this.setupConstants(FConstants, LConstants);
        this.initialize();
    }

    public Follower(HardwareMap hardwareMap, Localizer localizer, Class<?> FConstants, Class<?> LConstants) {
        this.hardwareMap = hardwareMap;
        this.setupConstants(FConstants, LConstants);
        this.initialize(localizer);
    }

    public void setupConstants(Class<?> FConstants, Class<?> LConstants) {
        Constants.setConstants(FConstants, LConstants);
        this.BEZIER_CURVE_SEARCH_LIMIT = FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT;
        this.AVERAGED_VELOCITY_SAMPLE_NUMBER = FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER;
        this.holdPointTranslationalScaling = FollowerConstants.holdPointTranslationalScaling;
        this.holdPointHeadingScaling = FollowerConstants.holdPointHeadingScaling;
        this.centripetalScaling = FollowerConstants.centripetalScaling;
        this.secondaryTranslationalPIDF = new PIDFController(FollowerConstants.secondaryTranslationalPIDFCoefficients);
        this.secondaryTranslationalIntegral = new PIDFController(FollowerConstants.secondaryTranslationalIntegral);
        this.translationalPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients);
        this.translationalIntegral = new PIDFController(FollowerConstants.translationalIntegral);
        this.secondaryHeadingPIDF = new PIDFController(FollowerConstants.secondaryHeadingPIDFCoefficients);
        this.headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
        this.secondaryDrivePIDF = new FilteredPIDFController(FollowerConstants.secondaryDrivePIDFCoefficients);
        this.drivePIDF = new FilteredPIDFController(FollowerConstants.drivePIDFCoefficients);
        this.driveKalmanFilter = new KalmanFilter(FollowerConstants.driveKalmanFilterParameters);
        this.turnHeadingErrorThreshold = FollowerConstants.turnHeadingErrorThreshold;
    }

    public void initialize() {
        this.poseUpdater = new PoseUpdater(this.hardwareMap);
        this.driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
        this.voltageSensor = (VoltageSensor)this.hardwareMap.voltageSensor.iterator().next();
        this.voltageTimer.reset();
        this.leftFront = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        this.leftRear = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        this.rightRear = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        this.rightFront = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        this.leftFront.setDirection(FollowerConstants.leftFrontMotorDirection);
        this.leftRear.setDirection(FollowerConstants.leftRearMotorDirection);
        this.rightFront.setDirection(FollowerConstants.rightFrontMotorDirection);
        this.rightRear.setDirection(FollowerConstants.rightRearMotorDirection);
        this.motors = Arrays.asList(this.leftFront, this.leftRear, this.rightFront, this.rightRear);

        for(DcMotorEx motor : this.motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction((double)1.0F);
            motor.setMotorType(motorConfigurationType);
        }

        this.setMotorsToFloat();
        this.dashboardPoseTracker = new DashboardPoseTracker(this.poseUpdater);
        this.breakFollowing();
    }

    public void initialize(Localizer localizer) {
        this.poseUpdater = new PoseUpdater(this.hardwareMap, localizer);
        this.driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
        this.voltageSensor = (VoltageSensor)this.hardwareMap.voltageSensor.iterator().next();
        this.voltageTimer.reset();
        this.leftFront = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        this.leftRear = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        this.rightRear = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        this.rightFront = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        this.leftFront.setDirection(FollowerConstants.leftFrontMotorDirection);
        this.leftRear.setDirection(FollowerConstants.leftRearMotorDirection);
        this.rightFront.setDirection(FollowerConstants.rightFrontMotorDirection);
        this.rightRear.setDirection(FollowerConstants.rightRearMotorDirection);
        this.motors = Arrays.asList(this.leftFront, this.leftRear, this.rightFront, this.rightRear);

        for(DcMotorEx motor : this.motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction((double)1.0F);
            motor.setMotorType(motorConfigurationType);
        }

        this.setMotorsToFloat();
        this.dashboardPoseTracker = new DashboardPoseTracker(this.poseUpdater);
        this.breakFollowing();
    }

    public void setCentripetalScaling(double set) {
        this.centripetalScaling = set;
    }

    private void setMotorsToBrake() {
        for(DcMotorEx motor : this.motors) {
            motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }

    }

    private void setMotorsToFloat() {
        for(DcMotorEx motor : this.motors) {
            motor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        }

    }

    public void setMaxPower(double set) {
        this.globalMaxPower = set;
        this.driveVectorScaler.setMaxPowerScaling(set);
    }

    public Point getPointFromPath(double t) {
        return this.currentPath != null ? this.currentPath.getPoint(t) : null;
    }

    public Pose getPose() {
        return this.poseUpdater.getPose();
    }

    public void setPose(Pose pose) {
        this.poseUpdater.setPose(pose);
    }

    public Vector getVelocity() {
        return this.poseUpdater.getVelocity();
    }

    public Vector getAcceleration() {
        return this.poseUpdater.getAcceleration();
    }

    public double getVelocityMagnitude() {
        return this.poseUpdater.getVelocity().getMagnitude();
    }

    public void setStartingPose(Pose pose) {
        this.poseUpdater.setStartingPose(pose);
    }

    public void setCurrentPoseWithOffset(Pose set) {
        this.poseUpdater.setCurrentPoseWithOffset(set);
    }

    public void setXOffset(double xOffset) {
        this.poseUpdater.setXOffset(xOffset);
    }

    public void setYOffset(double yOffset) {
        this.poseUpdater.setYOffset(yOffset);
    }

    public void setHeadingOffset(double headingOffset) {
        this.poseUpdater.setHeadingOffset(headingOffset);
    }

    public double getXOffset() {
        return this.poseUpdater.getXOffset();
    }

    public double getYOffset() {
        return this.poseUpdater.getYOffset();
    }

    public double getHeadingOffset() {
        return this.poseUpdater.getHeadingOffset();
    }

    public void resetOffset() {
        this.poseUpdater.resetOffset();
    }

    public void holdPoint(BezierPoint point, double heading) {
        this.breakFollowing();
        this.holdingPosition = true;
        this.isBusy = false;
        this.followingPathChain = false;
        this.currentPath = new Path(point);
        this.currentPath.setConstantHeadingInterpolation(heading);
        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), 1);
    }

    public void holdPoint(Point point, double heading) {
        this.holdPoint(new BezierPoint(point), heading);
    }

    public void holdPoint(Pose pose) {
        this.holdPoint(new Point(pose), pose.getHeading());
    }

    public void followPath(Path path, boolean holdEnd) {
        this.driveVectorScaler.setMaxPowerScaling(this.globalMaxPower);
        this.breakFollowing();
        this.holdPositionAtEnd = holdEnd;
        this.isBusy = true;
        this.followingPathChain = false;
        this.currentPath = path;
        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_SEARCH_LIMIT);
    }

    public void followPath(Path path) {
        this.followPath(path, FollowerConstants.automaticHoldEnd);
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        this.followPath(pathChain, this.globalMaxPower, holdEnd);
    }

    public void followPath(PathChain pathChain) {
        this.followPath(pathChain, FollowerConstants.automaticHoldEnd);
    }

    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        this.driveVectorScaler.setMaxPowerScaling(maxPower);
        this.breakFollowing();
        this.holdPositionAtEnd = holdEnd;
        this.pathStartTimes = new long[pathChain.size()];
        this.pathStartTimes[0] = System.currentTimeMillis();
        this.isBusy = true;
        this.followingPathChain = true;
        this.chainIndex = 0;
        this.currentPathChain = pathChain;
        this.currentPath = pathChain.getPath(this.chainIndex);
        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_SEARCH_LIMIT);
        this.currentPathChain.resetCallbacks();
    }

    public void resumePathFollowing() {
        this.pathStartTimes = new long[this.currentPathChain.size()];
        this.pathStartTimes[0] = System.currentTimeMillis();
        this.isBusy = true;
        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_SEARCH_LIMIT);
    }

    public void startTeleopDrive() {
        this.breakFollowing();
        this.teleopDrive = true;
        if (FollowerConstants.useBrakeModeInTeleOp) {
            this.setMotorsToBrake();
        }

    }

    public void updatePose() {
        this.poseUpdater.update();
        if (drawOnDashboard) {
            this.dashboardPoseTracker.update();
        }

    }

    public void update() {
        this.updatePose();
        if (!this.teleopDrive) {
            if (this.currentPath != null) {
                if (this.holdingPosition) {
                    this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), 1);
                    this.drivePowers = this.driveVectorScaler.getDrivePowers(MathFunctions.scalarMultiplyVector(this.getTranslationalCorrection(), this.holdPointTranslationalScaling), MathFunctions.scalarMultiplyVector(this.getHeadingVector(), this.holdPointHeadingScaling), new Vector(), this.poseUpdater.getPose().getHeading());

                    for(int i = 0; i < this.motors.size(); ++i) {
                        if (Math.abs(((DcMotorEx)this.motors.get(i)).getPower() - this.drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                            double voltageNormalized = this.getVoltageNormalized();
                            if (FollowerConstants.useVoltageCompensationInAuto) {
                                ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i] * voltageNormalized);
                            } else {
                                ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i]);
                            }
                        }
                    }

                    if (this.headingError < this.turnHeadingErrorThreshold && this.isTurning) {
                        this.isTurning = false;
                        this.isBusy = false;
                    }
                } else {
                    if (this.isBusy) {
                        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_SEARCH_LIMIT);
                        if (this.followingPathChain) {
                            this.updateCallbacks();
                        }

                        this.drivePowers = this.driveVectorScaler.getDrivePowers(this.getCorrectiveVector(), this.getHeadingVector(), this.getDriveVector(), this.poseUpdater.getPose().getHeading());

                        for(int i = 0; i < this.motors.size(); ++i) {
                            if (Math.abs(((DcMotorEx)this.motors.get(i)).getPower() - this.drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                                double voltageNormalized = this.getVoltageNormalized();
                                if (FollowerConstants.useVoltageCompensationInAuto) {
                                    ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i] * voltageNormalized);
                                } else {
                                    ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i]);
                                }
                            }
                        }
                    }

                    if (this.poseUpdater.getVelocity().getMagnitude() < (double)1.0F && this.currentPath.getClosestPointTValue() > 0.8 && this.zeroVelocityDetectedTimer == null && this.isBusy) {
                        this.zeroVelocityDetectedTimer = new ElapsedTime(Resolution.MILLISECONDS);
                        Log.d("Follower_logger", "!!!! Robot stuck !!!!");
                        this.debugLog();
                    }

                    if (this.currentPath.isAtParametricEnd() || this.zeroVelocityDetectedTimer != null && this.zeroVelocityDetectedTimer.milliseconds() > (double)500.0F) {
                        if (this.followingPathChain && this.chainIndex < this.currentPathChain.size() - 1) {
                            if (this.logDebug) {
                                Log.d("Follower_logger", "chainIndex: " + this.chainIndex + " | Pose: " + this.getPose());
                            }

                            this.breakFollowing();
                            this.pathStartTimes[this.chainIndex] = System.currentTimeMillis();
                            this.isBusy = true;
                            this.followingPathChain = true;
                            ++this.chainIndex;
                            this.currentPath = this.currentPathChain.getPath(this.chainIndex);
                            this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_SEARCH_LIMIT);
                        } else {
                            if (!this.reachedParametricPathEnd) {
                                this.reachedParametricPathEnd = true;
                                this.reachedParametricPathEndTime = System.currentTimeMillis();
                            }

                            if ((double)(System.currentTimeMillis() - this.reachedParametricPathEndTime) > this.currentPath.getPathEndTimeoutConstraint() || this.poseUpdater.getVelocity().getMagnitude() < this.currentPath.getPathEndVelocityConstraint() && MathFunctions.distance(this.poseUpdater.getPose(), this.closestPose) < this.currentPath.getPathEndTranslationalConstraint() && MathFunctions.getSmallestAngleDifference(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()) < this.currentPath.getPathEndHeadingConstraint()) {
                                if (this.holdPositionAtEnd) {
                                    this.holdPositionAtEnd = false;
                                    this.holdPoint(new BezierPoint(this.currentPath.getLastControlPoint()), this.currentPath.getHeadingGoal((double)1.0F));
                                } else {
                                    if (this.logDebug && this.isBusy) {
                                        Log.d("Follower_final_logger::", "isAtParametricEnd:" + this.currentPath.isAtParametricEnd() + " | isBusy: " + this.isBusy + " | closestPose:" + this.closestPose + " | Pose: " + this.getPose() + " | t-value: " + String.format("%3.5f", this.currentPath.getClosestPointTValue()) + " | velocity: " + String.format("%3.2f", this.poseUpdater.getVelocity().getMagnitude()) + " | distance: " + String.format("%3.2f", MathFunctions.distance(this.poseUpdater.getPose(), this.closestPose)) + " | heading (degree): " + String.format("%3.2f", Math.toDegrees(MathFunctions.getSmallestAngleDifference(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()))));
                                    }

                                    this.breakFollowing();
                                }
                            }
                        }
                    }
                }
            }
        } else {
            this.velocities.add(this.poseUpdater.getVelocity());
            this.velocities.remove(this.velocities.get(this.velocities.size() - 1));
            this.calculateAveragedVelocityAndAcceleration();
            this.drivePowers = this.driveVectorScaler.getDrivePowers(this.getCentripetalForceCorrection(), this.teleopHeadingVector, this.teleopDriveVector, this.poseUpdater.getPose().getHeading());

            for(int i = 0; i < this.motors.size(); ++i) {
                if (Math.abs(((DcMotorEx)this.motors.get(i)).getPower() - this.drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                    double voltageNormalized = this.getVoltageNormalized();
                    if (FollowerConstants.useVoltageCompensationInTeleOp) {
                        ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i] * voltageNormalized);
                    } else {
                        ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i]);
                    }
                }
            }
        }

    }

    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading) {
        this.setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, true);
    }

    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        this.teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, (double)-1.0F, (double)1.0F);
        this.teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, (double)-1.0F, (double)1.0F);
        this.teleopDriveValues[2] = MathFunctions.clamp(heading, (double)-1.0F, (double)1.0F);
        this.teleopDriveVector.setOrthogonalComponents(this.teleopDriveValues[0], this.teleopDriveValues[1]);
        this.teleopDriveVector.setMagnitude(MathFunctions.clamp(this.teleopDriveVector.getMagnitude(), (double)0.0F, (double)1.0F));
        if (robotCentric) {
            this.teleopDriveVector.rotateVector(this.getPose().getHeading());
        }

        this.teleopHeadingVector.setComponents(this.teleopDriveValues[2], this.getPose().getHeading());
    }

    public void calculateAveragedVelocityAndAcceleration() {
        this.averageVelocity = new Vector();
        this.averagePreviousVelocity = new Vector();

        for(int i = 0; i < this.velocities.size() / 2; ++i) {
            this.averageVelocity = MathFunctions.addVectors(this.averageVelocity, (Vector)this.velocities.get(i));
        }

        this.averageVelocity = MathFunctions.scalarMultiplyVector(this.averageVelocity, (double)1.0F / ((double)this.velocities.size() / (double)2.0F));

        for(int i = this.velocities.size() / 2; i < this.velocities.size(); ++i) {
            this.averagePreviousVelocity = MathFunctions.addVectors(this.averagePreviousVelocity, (Vector)this.velocities.get(i));
        }

        this.averagePreviousVelocity = MathFunctions.scalarMultiplyVector(this.averagePreviousVelocity, (double)1.0F / ((double)this.velocities.size() / (double)2.0F));
        this.accelerations.add(MathFunctions.subtractVectors(this.averageVelocity, this.averagePreviousVelocity));
        this.accelerations.remove(this.accelerations.size() - 1);
        this.averageAcceleration = new Vector();

        for(int i = 0; i < this.accelerations.size(); ++i) {
            this.averageAcceleration = MathFunctions.addVectors(this.averageAcceleration, (Vector)this.accelerations.get(i));
        }

        this.averageAcceleration = MathFunctions.scalarMultiplyVector(this.averageAcceleration, (double)1.0F / (double)this.accelerations.size());
    }

    public void updateCallbacks() {
        for(PathCallback callback : this.currentPathChain.getCallbacks()) {
            if (!callback.hasBeenRun()) {
                if (callback.getType() == 1) {
                    if (this.chainIndex == callback.getIndex() && (this.getCurrentTValue() >= callback.getStartCondition() || MathFunctions.roughlyEquals(this.getCurrentTValue(), callback.getStartCondition()))) {
                        callback.run();
                    }
                } else if (this.chainIndex >= callback.getIndex() && (double)(System.currentTimeMillis() - this.pathStartTimes[callback.getIndex()]) > callback.getStartCondition()) {
                    callback.run();
                }
            }
        }

    }

    public void breakFollowing() {
        this.teleopDrive = false;
        this.setMotorsToFloat();
        this.holdingPosition = false;
        this.isBusy = false;
        this.reachedParametricPathEnd = false;
        this.secondaryDrivePIDF.reset();
        this.drivePIDF.reset();
        this.secondaryHeadingPIDF.reset();
        this.headingPIDF.reset();
        this.secondaryTranslationalPIDF.reset();
        this.secondaryTranslationalIntegral.reset();
        this.secondaryTranslationalIntegralVector = new Vector();
        this.previousSecondaryTranslationalIntegral = (double)0.0F;
        this.translationalPIDF.reset();
        this.translationalIntegral.reset();
        this.translationalIntegralVector = new Vector();
        this.previousTranslationalIntegral = (double)0.0F;
        this.driveVector = new Vector();
        this.headingVector = new Vector();
        this.translationalVector = new Vector();
        this.centripetalVector = new Vector();
        this.correctiveVector = new Vector();
        this.driveError = (double)0.0F;
        this.headingError = (double)0.0F;
        this.rawDriveError = (double)0.0F;
        this.previousRawDriveError = (double)0.0F;
        this.driveErrors = new double[2];

        for(int i = 0; i < this.driveErrors.length; ++i) {
            this.driveErrors[i] = (double)0.0F;
        }

        this.driveKalmanFilter.reset();

        for(int i = 0; i < this.AVERAGED_VELOCITY_SAMPLE_NUMBER; ++i) {
            this.velocities.add(new Vector());
        }

        for(int i = 0; i < this.AVERAGED_VELOCITY_SAMPLE_NUMBER / 2; ++i) {
            this.accelerations.add(new Vector());
        }

        this.calculateAveragedVelocityAndAcceleration();
        this.teleopDriveValues = new double[3];
        this.teleopDriveVector = new Vector();
        this.teleopHeadingVector = new Vector();

        for(int i = 0; i < this.motors.size(); ++i) {
            ((DcMotorEx)this.motors.get(i)).setPower((double)0.0F);
        }

        this.zeroVelocityDetectedTimer = null;
    }

    public boolean isBusy() {
        return this.isBusy;
    }

    public Vector getDriveVector() {
        if (!useDrive) {
            return new Vector();
        } else if (this.followingPathChain && this.chainIndex < this.currentPathChain.size() - 1) {
            return new Vector(this.driveVectorScaler.getMaxPowerScaling(), this.currentPath.getClosestPointTangentVector().getTheta());
        } else {
            this.driveError = this.getDriveVelocityError();
            if (Math.abs(this.driveError) < FollowerConstants.drivePIDFSwitch && FollowerConstants.useSecondaryDrivePID) {
                this.secondaryDrivePIDF.updateError(this.driveError);
                this.driveVector = new Vector(MathFunctions.clamp(this.secondaryDrivePIDF.runPIDF() + FollowerConstants.secondaryDrivePIDFFeedForward * MathFunctions.getSign(this.driveError), -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.currentPath.getClosestPointTangentVector().getTheta());
                return MathFunctions.copyVector(this.driveVector);
            } else {
                this.drivePIDF.updateError(this.driveError);
                this.driveVector = new Vector(MathFunctions.clamp(this.drivePIDF.runPIDF() + FollowerConstants.drivePIDFFeedForward * MathFunctions.getSign(this.driveError), -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.currentPath.getClosestPointTangentVector().getTheta());
                return MathFunctions.copyVector(this.driveVector);
            }
        }
    }

    public double getDriveVelocityError() {
        double distanceToGoal;
        if (!this.currentPath.isAtParametricEnd()) {
            distanceToGoal = this.currentPath.length() * ((double)1.0F - this.currentPath.getClosestPointTValue());
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(this.getPose().getX() - this.currentPath.getLastControlPoint().getX(), this.getPose().getY() - this.currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(this.currentPath.getEndTangent(), offset);
        }

        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(this.getVelocity(), MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), this.currentPath.getClosestPointTangentVector().getTheta());
        Vector forwardHeadingVector = new Vector((double)1.0F, this.poseUpdater.getPose().getHeading());
        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs((double)-2.0F * this.currentPath.getZeroPowerAccelerationMultiplier() * FollowerConstants.forwardZeroPowerAcceleration * (double)(forwardDistanceToGoal <= (double)0.0F ? 1 : -1) * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, (double)2.0F) + (double)2.0F * FollowerConstants.forwardZeroPowerAcceleration * Math.abs(forwardDistanceToGoal)));
        Vector lateralHeadingVector = new Vector((double)1.0F, this.poseUpdater.getPose().getHeading() - (Math.PI / 2D));
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);
        double lateralVelocityGoal = MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs((double)-2.0F * this.currentPath.getZeroPowerAccelerationMultiplier() * FollowerConstants.lateralZeroPowerAcceleration * (double)(lateralDistanceToGoal <= (double)0.0F ? 1 : -1) * lateralDistanceToGoal));
        double lateralVelocityZeroPowerDecay = lateralVelocity - MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(lateralVelocity, (double)2.0F) + (double)2.0F * FollowerConstants.lateralZeroPowerAcceleration * Math.abs(lateralDistanceToGoal)));
        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);
        this.previousRawDriveError = this.rawDriveError;
        this.rawDriveError = velocityErrorVector.getMagnitude() * MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, this.currentPath.getClosestPointTangentVector()));
        double projection = (double)2.0F * this.driveErrors[1] - this.driveErrors[0];
        this.driveKalmanFilter.update(this.rawDriveError - this.previousRawDriveError, projection);

        for(int i = 0; i < this.driveErrors.length - 1; ++i) {
            this.driveErrors[i] = this.driveErrors[i + 1];
        }

        this.driveErrors[1] = this.driveKalmanFilter.getState();
        return this.driveKalmanFilter.getState();
    }

    public Vector getHeadingVector() {
        if (!useHeading) {
            return new Vector();
        } else {
            this.headingError = MathFunctions.getTurnDirection(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal());
            if (Math.abs(this.headingError) < FollowerConstants.headingPIDFSwitch && FollowerConstants.useSecondaryHeadingPID) {
                this.secondaryHeadingPIDF.updateError(this.headingError);
                this.headingVector = new Vector(MathFunctions.clamp(this.secondaryHeadingPIDF.runPIDF() + FollowerConstants.secondaryHeadingPIDFFeedForward * MathFunctions.getTurnDirection(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()), -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.poseUpdater.getPose().getHeading());
                return MathFunctions.copyVector(this.headingVector);
            } else {
                this.headingPIDF.updateError(this.headingError);
                this.headingVector = new Vector(MathFunctions.clamp(this.headingPIDF.runPIDF() + FollowerConstants.headingPIDFFeedForward * MathFunctions.getTurnDirection(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()), -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.poseUpdater.getPose().getHeading());
                return MathFunctions.copyVector(this.headingVector);
            }
        }
    }

    public Vector getCorrectiveVector() {
        Vector centripetal = this.getCentripetalForceCorrection();
        Vector translational = this.getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);
        if (corrective.getMagnitude() > this.driveVectorScaler.getMaxPowerScaling()) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, this.driveVectorScaler.findNormalizingScaling(centripetal, translational)));
        } else {
            this.correctiveVector = MathFunctions.copyVector(corrective);
            return corrective;
        }
    }

    public Vector getTranslationalCorrection() {
        if (!useTranslational) {
            return new Vector();
        } else {
            Vector translationalVector = new Vector();
            double x = this.closestPose.getX() - this.poseUpdater.getPose().getX();
            double y = this.closestPose.getY() - this.poseUpdater.getPose().getY();
            translationalVector.setOrthogonalComponents(x, y);
            if (!this.currentPath.isAtParametricEnd() && !this.currentPath.isAtParametricStart()) {
                translationalVector = MathFunctions.subtractVectors(translationalVector, new Vector(MathFunctions.dotProduct(translationalVector, MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), this.currentPath.getClosestPointTangentVector().getTheta()));
                this.secondaryTranslationalIntegralVector = MathFunctions.subtractVectors(this.secondaryTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(this.secondaryTranslationalIntegralVector, MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), this.currentPath.getClosestPointTangentVector().getTheta()));
                this.translationalIntegralVector = MathFunctions.subtractVectors(this.translationalIntegralVector, new Vector(MathFunctions.dotProduct(this.translationalIntegralVector, MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), this.currentPath.getClosestPointTangentVector().getTheta()));
            }

            if (MathFunctions.distance(this.poseUpdater.getPose(), this.closestPose) < FollowerConstants.translationalPIDFSwitch && FollowerConstants.useSecondaryTranslationalPID) {
                this.secondaryTranslationalIntegral.updateError(translationalVector.getMagnitude());
                this.secondaryTranslationalIntegralVector = MathFunctions.addVectors(this.secondaryTranslationalIntegralVector, new Vector(this.secondaryTranslationalIntegral.runPIDF() - this.previousSecondaryTranslationalIntegral, translationalVector.getTheta()));
                this.previousSecondaryTranslationalIntegral = this.secondaryTranslationalIntegral.runPIDF();
                this.secondaryTranslationalPIDF.updateError(translationalVector.getMagnitude());
                translationalVector.setMagnitude(this.secondaryTranslationalPIDF.runPIDF() + FollowerConstants.secondaryTranslationalPIDFFeedForward);
                translationalVector = MathFunctions.addVectors(translationalVector, this.secondaryTranslationalIntegralVector);
            } else {
                this.translationalIntegral.updateError(translationalVector.getMagnitude());
                this.translationalIntegralVector = MathFunctions.addVectors(this.translationalIntegralVector, new Vector(this.translationalIntegral.runPIDF() - this.previousTranslationalIntegral, translationalVector.getTheta()));
                this.previousTranslationalIntegral = this.translationalIntegral.runPIDF();
                this.translationalPIDF.updateError(translationalVector.getMagnitude());
                translationalVector.setMagnitude(this.translationalPIDF.runPIDF() + FollowerConstants.translationalPIDFFeedForward);
                translationalVector = MathFunctions.addVectors(translationalVector, this.translationalIntegralVector);
            }

            translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), (double)0.0F, this.driveVectorScaler.getMaxPowerScaling()));
            this.translationalVector = MathFunctions.copyVector(translationalVector);
            return translationalVector;
        }
    }

    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = this.closestPose.getX() - this.poseUpdater.getPose().getX();
        double y = this.closestPose.getY() - this.poseUpdater.getPose().getY();
        error.setOrthogonalComponents(x, y);
        return error;
    }

    public Vector getCentripetalForceCorrection() {
        if (!useCentripetal) {
            return new Vector();
        } else {
            double curvature;
            if (!this.teleopDrive) {
                curvature = this.currentPath.getClosestPointCurvature();
            } else {
                double yPrime = this.averageVelocity.getYComponent() / this.averageVelocity.getXComponent();
                double yDoublePrime = this.averageAcceleration.getYComponent() / this.averageVelocity.getXComponent();
                curvature = yDoublePrime / Math.pow(Math.sqrt((double)1.0F + Math.pow(yPrime, (double)2.0F)), (double)3.0F);
            }

            if (Double.isNaN(curvature)) {
                return new Vector();
            } else {
                this.centripetalVector = new Vector(MathFunctions.clamp(this.centripetalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(this.poseUpdater.getVelocity(), MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), (double)2.0F) * curvature, -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.currentPath.getClosestPointTangentVector().getTheta() + (Math.PI / 2D) * MathFunctions.getSign(this.currentPath.getClosestPointNormalVector().getTheta()));
                return this.centripetalVector;
            }
        }
    }

    public Pose getClosestPose() {
        return this.closestPose;
    }

    public boolean atParametricEnd() {
        if (this.followingPathChain) {
            return this.chainIndex == this.currentPathChain.size() - 1 ? this.currentPath.isAtParametricEnd() : false;
        } else {
            return this.currentPath.isAtParametricEnd();
        }
    }

    public double getCurrentTValue() {
        return this.isBusy ? this.currentPath.getClosestPointTValue() : (double)1.0F;
    }

    public double getCurrentPathNumber() {
        return !this.followingPathChain ? (double)0.0F : (double)this.chainIndex;
    }

    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }

//    public void telemetryDebug(MultipleTelemetry telemetry) {
//        telemetry.addData("follower busy", this.isBusy());
//        telemetry.addData("heading error", this.headingError);
//        telemetry.addData("heading vector magnitude", this.headingVector.getMagnitude());
//        telemetry.addData("corrective vector magnitude", this.correctiveVector.getMagnitude());
//        telemetry.addData("corrective vector heading", this.correctiveVector.getTheta());
//        telemetry.addData("translational error magnitude", this.getTranslationalError().getMagnitude());
//        telemetry.addData("translational error direction", this.getTranslationalError().getTheta());
//        telemetry.addData("translational vector magnitude", this.translationalVector.getMagnitude());
//        telemetry.addData("translational vector heading", this.translationalVector.getMagnitude());
//        telemetry.addData("centripetal vector magnitude", this.centripetalVector.getMagnitude());
//        telemetry.addData("centripetal vector heading", this.centripetalVector.getTheta());
//        telemetry.addData("drive error", this.driveError);
//        telemetry.addData("drive vector magnitude", this.driveVector.getMagnitude());
//        telemetry.addData("drive vector heading", this.driveVector.getTheta());
//        telemetry.addData("x", this.getPose().getX());
//        telemetry.addData("y", this.getPose().getY());
//        telemetry.addData("heading", this.getPose().getHeading());
//        telemetry.addData("total heading", this.poseUpdater.getTotalHeading());
//        telemetry.addData("velocity magnitude", this.getVelocity().getMagnitude());
//        telemetry.addData("velocity heading", this.getVelocity().getTheta());
//        this.driveKalmanFilter.debug(telemetry);
//        telemetry.update();
//        if (drawOnDashboard) {
//            Drawing.drawDebug(this);
//        }
//
//    }

    public void telemetryDebug(Telemetry telemetry) {
        this.telemetryDebug(new MultipleTelemetry(new Telemetry[]{telemetry}));
    }

    public double getTotalHeading() {
        return this.poseUpdater.getTotalHeading();
    }

    public Path getCurrentPath() {
        return this.currentPath;
    }

    public DashboardPoseTracker getDashboardPoseTracker() {
        return this.dashboardPoseTracker;
    }

    private void resetIMU() throws InterruptedException {
        this.poseUpdater.resetIMU();
    }

    private void debugLog() {
        Log.d("Follower_logger::", "isAtParametricEnd:" + this.currentPath.isAtParametricEnd() + " | isBusy: " + this.isBusy + " | closestPose:" + this.closestPose + " | Pose: " + this.getPose() + " | t-value: " + String.format("%3.5f", this.currentPath.getClosestPointTValue()) + " | zeroVelocityTimer: " + String.format("%3.2f", this.zeroVelocityDetectedTimer == null ? (double)0.0F : this.zeroVelocityDetectedTimer.milliseconds()) + " | velocity: " + String.format("%3.2f", this.poseUpdater.getVelocity().getMagnitude()) + " | distance: " + String.format("%3.2f", MathFunctions.distance(this.poseUpdater.getPose(), this.closestPose)) + " | heading (degree): " + String.format("%3.2f", Math.toDegrees(MathFunctions.getSmallestAngleDifference(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()))));
    }

    public boolean isRobotStuck() {
        return this.zeroVelocityDetectedTimer != null;
    }

//    public void drawOnDashBoard() {
//        if (drawOnDashboard) {
//            Drawing.drawDebug(this);
//        }
//
//    }

    public boolean isLocalizationNAN() {
        return this.poseUpdater.getLocalizer().isNAN();
    }

    public double getVoltage() {
        if (this.voltageTimer.seconds() > FollowerConstants.cacheInvalidateSeconds && FollowerConstants.cacheInvalidateSeconds >= (double)0.0F) {
            this.cached = false;
        }

        if (!this.cached) {
            this.refreshVoltage();
        }

        return this.voltage;
    }

    public double getVoltageNormalized() {
        return Math.min(FollowerConstants.nominalVoltage / this.getVoltage(), (double)1.0F);
    }

    public void refreshVoltage() {
        this.cached = true;
        this.voltage = this.voltageSensor.getVoltage();
        this.voltageTimer.reset();
    }

    public void turn(double radians, boolean isLeft) {
        Pose temp = new Pose(this.getPose().getX(), this.getPose().getY(), this.getPose().getHeading() + (isLeft ? radians : -radians));
        this.holdPoint(temp);
        this.isTurning = true;
        this.isBusy = true;
    }

    public void turnTo(double radians) {
        this.holdPoint(new Pose(this.getPose().getX(), this.getPose().getY(), Math.toRadians(radians)));
        this.isTurning = true;
        this.isBusy = true;
    }

    public void turnToDegrees(double degrees) {
        this.turnTo(Math.toRadians(degrees));
    }

    public void turnDegrees(double degrees, boolean isLeft) {
        this.turn(Math.toRadians(degrees), isLeft);
    }

    public boolean isTurning() {
        return this.isTurning;
    }

    public void setHeadingPIDF(CustomPIDFCoefficients set) {
        this.headingPIDF.setCoefficients(set);
    }

    public void setTranslationalPIDF(CustomPIDFCoefficients set) {
        this.translationalPIDF.setCoefficients(set);
    }

    public void setDrivePIDF(CustomFilteredPIDFCoefficients set) {
        this.drivePIDF.setCoefficients(set);
    }

    public void setSecondaryHeadingPIDF(CustomPIDFCoefficients set) {
        this.secondaryHeadingPIDF.setCoefficients(set);
    }

    public void setSecondaryTranslationalPIDF(CustomPIDFCoefficients set) {
        this.secondaryTranslationalPIDF.setCoefficients(set);
    }

    public void setSecondaryDrivePIDF(CustomFilteredPIDFCoefficients set) {
        this.secondaryDrivePIDF.setCoefficients(set);
    }

    public boolean atPoint(Point point, double xTolerance, double yTolerance) {
        return Math.abs(point.getX() - this.getPose().getX()) < xTolerance && Math.abs(point.getY() - this.getPose().getY()) < yTolerance;
    }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return Math.abs(pose.getX() - this.getPose().getX()) < xTolerance && Math.abs(pose.getY() - this.getPose().getY()) < yTolerance && Math.abs(pose.getHeading() - this.getPose().getHeading()) < headingTolerance;
    }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
        return Math.abs(pose.getX() - this.getPose().getX()) < xTolerance && Math.abs(pose.getY() - this.getPose().getY()) < yTolerance;
    }

    public double getHeadingError() {
        return this.headingError;
    }
}
