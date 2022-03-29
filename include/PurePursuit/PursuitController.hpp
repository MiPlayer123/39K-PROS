#pragma once

#include "PurePursuit/purePursuitPath.hpp"
#include "PurePursuit/purePursuitController.hpp"
#include "PurePursuit/geometry/pose2d.hpp"
#include "PurePursuit/geometry/rotation2d.hpp"
#include "PurePursuit/geometry/translation2d.hpp"
#include "PurePursuit/util/mathUtil.hpp"
#include "PurePursuit/util/plotter.hpp"
#include "PurePursuit/util/slewRateLimiter.hpp"
#include "PurePursuit/util/taskWrapper.hpp"

#include "okapi/api.hpp"

#define makeSettlableLoop(settleFunction, itimeout, iunitsError, loopedBody) \
    uint32_t start = pros::millis();                            \
    uint32_t timeElapsed = pros::millis() - start;              \
    bool timeLeft = (timeElapsed < itimeout);                   \
    bool settled = settleFunction();                            \
    auto rate = timeUtil.getRate();                             \
    while (!settled && timeLeft && task->notifyTake(0) == 0U) { \
        loopedBody                                              \
        timeElapsed = pros::millis() - start;                   \
        settled = settleFunction();                             \
        timeLeft = (timeElapsed < itimeout);                    \
        rate->delayUntil(10_ms);                                \
    }

#define parseTimeout(timeoutVar)  \
    if (timeoutVar == 0) {        \
        timeoutVar = ~timeoutVar; \
    }

using namespace okapi;

class pursuitController : public TaskWrapper {
public:
    enum class TurnType {
        LeftPivot, PointTurn, RightPivot
    };

    /**
     * X-drive chassis control using odometry.
     *
     * @param itimeUtil The TimeUtil.
     * @param imodel The XDriveModel used to read from sensors/write to motors.
     * @param idistanceThreshold minimum length movement (smaller movements will be skipped)
     * @param iturnThreshold minimum angle turn (smaller turns will be skipped)
     */
    pursuitController(
        TimeUtil itimeUtil,
        std::shared_ptr<OdomChassisController> imodel,
        QLength idistanceThreshold = 0_mm,
        QAngle iturnThreshold = 0_deg);

    pursuitController(const pursuitController&) = delete;
    pursuitController(pursuitController&& other) = delete;
    pursuitController &operator=(const pursuitController& other) = delete;
    pursuitController &operator=(pursuitController&& other) = delete;
    ~pursuitController() override;

    /**
     * Makes the robot follow a path using Pure Pursuit.
     *
     * @param ipath The path to follow.
     * @param ilookahead The lookahead radius.
     * @param isettleRadius The settle radius.
     * @param igains The velocity PID & feedforward gains.
     */
    void followPath(const std::shared_ptr<PurePursuitPath>& ipath, 
        QLength ilookahead, QLength isettleRadius,
        PurePursuitController::Gains igains, int itimeout = 0
    );

    /**
     * Sets the current odometry state.
     *
     * @param istate the new state
     */
    void setState(OdomState istate);

    void setPose(const Pose2d& ipose);

    /**
     * Returns the current odometry state.
     *
     * @return the odometry state
     */
    OdomState getState();

    /**
     * Stops the chassis and disables all PID controllers.
     */
    void stop();

    /**
     * Gets the ChassisScales.
     */
    ChassisScales getChassisScales() const;

    /**
     * Blocks the thread until the odometry task is started.
     */
    void waitForOdomTask();
protected:
    std::shared_ptr<OdomChassisController> model {nullptr};

    std::unique_ptr<PurePursuitController> purePursuitController {nullptr};
    std::unique_ptr<SlewRateLimiter> targetVelocitySlewRate {nullptr};

    ChassisScales scales;

    QLength distanceThreshold;
    QAngle turnThreshold;

    TimeUtil timeUtil;

    std::atomic_bool dtorCalled {false};

    /**
     * Stops all the controllers and the ChassisModel.
     */
    void stopAfterSettled();
};