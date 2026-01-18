#pragma once

#include <frc/geometry/Translation2d.h>

#include "lib/math/QuarticSolver.h"

struct Result 
{
    double ShotSpeed;
    double TravelTime;
    frc::Translation2d AimPosition;
};

class ShotPredictor 
{
    private:
        units::radian_t MinAngle;
        units::radian_t MaxAngle;
        double MaxAngleVelocity;
        double DesiredShotVelocity;

        static const int NewtonRepetitions = 8;
        static const double maxNewtonError = 0.1;
        static const double gravity = -9.8;

        static const int updateSteps = 5; // divides the possible angle range
        static const int updateRepetitions = 5; // repeats the process like a binary search

    public:

        ShotPredictor(units::radian_t minAngle, units::radian_t maxAngle, double maxAngleVelocity, double desiredShotVelocity) {
            MinAngle = minAngle;
            MaxAngle = maxAngle;
            MaxAngleVelocity = maxAngleVelocity;
            DesiredShotVelocity = desiredShotVelocity;
        }

        static std::optional<Result> Predict(units::radian_t shotAngle, frc::Translation2d botVelocity, double verticalVelocity, frc::Translation2d relativeTargetPosition, double targetHeight) {
            // If you want to understand the math (kinda)
            // https://www.overleaf.com/read/dfcbwtvytpms#32640b
            auto botSpeedSq = 1_m * std::pow(botVelocity.Distance(frc::Translation2d{}).value(), 2);
            auto targetDistance = relativeTargetPosition.Distance(frc::Translation2d{});
            auto dot = -relativeTargetPosition.Dot(botVelocity);
            auto tansq = 1_rad * std::pow(std::tan(shotAngle.value()), 2);

            double guess = (verticalVelocity + std::sqrt(std::pow(verticalVelocity, 2) - 2 * gravity * targetDistance.value() * tansq.value())) / (-gravity) + 2;

            auto quarticResult = SolveP5(
                guess,
                0.25 * std::pow(gravity, 2),
                verticalVelocity * gravity,
                std::pow(verticalVelocity, 2) - targetHeight * gravity - tansq.value() * botSpeedSq.value(),
                -(2 * targetHeight * verticalVelocity + 2 * tansq * dot),
                std::pow(targetHeight, 2) - tansq.value() * std::pow(targetDistance.value(), 2)
            );

            if (!quarticResult.has_value()) {
                return std::optional<Result>{};
            }

            double travelTime = quarticResult.value();

            if (travelTime <= 0) {
                return std::optional<Result>{};
            }

            Result result;
            result.TravelTime = travelTime;
            result.ShotSpeed =
                (std::sqrt(botSpeedSq * travelTime * travelTime + std::pow(targetDistance.value(), 2) + 2 * travelTime * dot.value()))
                /(travelTime * std::cos(shotAngle.value()));
            result.AimPosition = relativeTargetPosition + (botVelocity * -travelTime);

            return std::optional<Result>{result};
        }

        std::optional<Result> Update(bool loose, units::radian_t currentAngle, double deltaTime, frc::Translation2d botVelocity, double verticalVelocity, frc::Translation2d relativeTargetPosition, double targetHeight) {
            units::radian_t min = 1_rad * (loose ? MinAngle.value() : std::max(MinAngle.value(), currentAngle.value() - deltaTime * MaxAngleVelocity));
            units::radian_t max = 1_rad * (loose ? MaxAngle.value() : std::min(MaxAngle.value(), currentAngle.value() + deltaTime * MaxAngleVelocity));

            units::radian_t finalAngle = currentAngle;
            std::unordered_map<units::radian_t, std::optional<Result>> cache = {{}};

            // repetitively close in on the best angle by repeating the process
            for (int i = 0; i < updateRepetitions; i ++) 
            {    
                units::radian_t bestAngle{0};
                units::radian_t bestErr  {999999}; // was MAX_VALUE

                // divide the interval into sections and find the best one
                units::radian_t halfStepSize = (max - min)/(updateSteps*2);
                for (units::radian_t angle = min + halfStepSize; angle < max; angle += 2 * halfStepSize) 
                {
                    std::optional<Result> resultOpt = cache.contains(angle)
                        ? cache[angle]
                        : Predict(angle, botVelocity, verticalVelocity, relativeTargetPosition, targetHeight);

                    cache.emplace(angle, resultOpt);

                    if (!resultOpt.has_value()) {
                        continue;
                    }

                    Result result = resultOpt.value();

                    units::radian_t err = 1_rad * std::abs(DesiredShotVelocity - result.ShotSpeed);
                    if (err < bestErr) {
                        bestErr = err;
                        bestAngle = angle;
                    }
                }

                // no options, so fail
                if (bestAngle == 999999_rad) {
                    return std::optional<Result>{};
                }

                // repeat with a new interval closed in around the best option
                min = bestAngle - halfStepSize;
                finalAngle = bestAngle;
                max = bestAngle + halfStepSize;
            }

            return cache[finalAngle];
        }
};