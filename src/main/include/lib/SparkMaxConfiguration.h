#pragma once 

#pragma region Includes
#include <iostream>
#include <numbers>

#include <units/angle.h>
#include <units/voltage.h>

#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkLowLevel.h>
#include <rev/config/SparkMaxConfig.h>
#pragma endregion

#pragma region SparkMaxConfiguration
void SparkMaxConfiguration(rev::spark::SparkMax *motor,
                          units::ampere_t currentLimit,
                          bool   breakMode,
                          double P,
                          double I,
                          double D,
                          double S,
                          double V,
                          double A,
                          units::turns_per_second_t         velocityLimit,
                          units::turns_per_second_squared_t accelerationLimit);
#pragma endregion