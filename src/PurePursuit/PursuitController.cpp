#include <atomic>
#include <memory>
#include <utility>

#include "PursuitController.hpp"
#include "PurePursuit/purePursuitPath.hpp"
#include "PurePursuit/purePursuitController.hpp"
#include "PurePursuit/geometry/pose2d.hpp"
#include "PurePursuit/geometry/rotation2d.hpp"
#include "PurePursuit/geometry/translation2d.hpp"
#include "PurePursuit/util/mathUtil.hpp"
#include "PurePursuit/util/plotter.hpp"
#include "PurePursuit/util/slewRateLimiter.hpp"
#include "okapi/api.hpp"
/*
pursuitController::pursuitController(
    TimeUtil itimeUtil,
    std::shared_ptr<OdomChassisController> model, //imodel
    QLength idistanceThreshold,
    QAngle iturnThreshold
) : timeUtil{std::move(itimeUtil)},
    //model{std::move(imodel)},
    targetVelocitySlewRate{std::make_unique<SlewRateLimiter>(0.0, 0.0, 0.0)},
    purePursuitController{std::make_unique<PurePursuitController>()}
    
{}

pursuitController::~pursuitController() {
    dtorCalled.store(true, std::memory_order_release);
}

void pursuitController::followPath(const std::shared_ptr<PurePursuitPath>& ipath, 
    QLength ilookahead, QLength isettleRadius, PurePursuitController::Gains igains, int itimeout
) {
    purePursuitController->setPath(ipath, ilookahead);
    targetVelocitySlewRate->reset(0.0);
    targetVelocitySlewRate->setLimits(ipath->constraints.maxAcceleration);

    Pose2d currentPose = Pose2d::fromOdomState(getState());
    Translation2d currentPosition = currentPose.translation();
    Translation2d previousPosition = currentPosition;
    double currentVelocity = 0.0;
    double targetVelocity = 0.0;
    double previousTargetVelocity = 0.0;
    double targetAcceleration = 0.0;
    double output = 0.0;
    PurePursuitPath::Point lookaheadPoint = {};

    double distanceError = currentPosition.distance(
        ipath->getLastPoint().pose.translation()
    ).convert(meter);

    auto isFollowingSettled = [&]() -> bool {
        std::cout << "Current error: " << distanceError << " m" << std::endl;
        return distanceError <= isettleRadius.convert(meter);
    };

    plotterStart();
    auto startTime = pros::millis();
    makeSettlableLoop(isFollowingSettled, itimeout, distanceError, {
        currentPose = Pose2d::fromOdomState(getState());
        currentPosition = currentPose.translation();
        currentVelocity = currentPosition.distance(previousPosition).convert(meter) / (10_ms).convert(second);
        lookaheadPoint = purePursuitController->calculate(currentPose);
        targetVelocity = targetVelocitySlewRate->calculate(lookaheadPoint.targetVelocity);

        targetAcceleration = targetVelocity - previousTargetVelocity;

        plotterPlot({(pros::millis() - startTime) / 1000.0, targetVelocity, currentVelocity});

        // PID w/ FF
        output = igains.kP * (targetVelocity - currentVelocity) 
            + igains.kV * targetVelocity + igains.kA * targetAcceleration;

        std::cout << "Target Vel: " << targetVelocity << " m/s | Current Vel: " << currentVelocity << " m/s | Target Acceleration: " << targetAcceleration << std::endl;

        // Find the direction the drive should move towards in global coordinates
        auto directionVector = lookaheadPoint.pose.translation() - currentPosition;

        // Use the same vector to find the distance to the target
        double distance = directionVector.norm().convert(meter);

        // QAngle gyroRotation = -currentPose.angle();
        QAngle gyroRotation = -model->getState().theta.convert(degrees) * degree;

        // Normalize the vector & scale it by the PID output
        directionVector /= distance;
        directionVector *= std::clamp(output, -1.0, 1.0);
        directionVector = directionVector.rotateBy(Rotation2d{-gyroRotation});

        /*
        model->xArcade(
            directionVector.x().convert(meter),
            directionVector.y().convert(meter), 
            0.0
        );
        */
/*
        distanceError = currentPosition.distance(
            ipath->getLastPoint().pose.translation()
        ).convert(meter);

        previousPosition = currentPosition;
        previousTargetVelocity = targetVelocity;
    });
    plotterStop();
    stopAfterSettled();
}

void pursuitController::setState(OdomState istate) {
    model->setState(istate);
}

void pursuitController::setPose(const Pose2d &ipose) {
    model->setState({
        ipose.x(), ipose.y(), 
        constrainAnglePi(-ipose.angle().convert(radian)) * radian
    });
}

OdomState pursuitController::getState() {
    return model->getState();
}

ChassisScales pursuitController::getChassisScales() const {
    return scales;
}

void pursuitController::stopAfterSettled() {
    model->stop();
}
*/