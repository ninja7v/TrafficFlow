// Librairies
#include <gtest/gtest.h>  // for Google Test framework
#include <cmath>          // for std::abs
#include <cstdio>         // for std::remove
#include <fstream>        // for std::ifstream
#include <sstream>        // for std::stringstream
#include <memory>         // for std::make_unique and std::make_shared
#include <list>           // for std::list
#include <array>          // for std::array
#include <vector>         // for std::vector
#include <string>         // for std::string
// Headers
#include "../headers/Bike.h"
#include "../headers/Car.h"
#include "../headers/Truck.h"
#include "../headers/Constants.h"
#include "../headers/Controllers.h"
#include "../headers/Features.h"
#include "../headers/Graph.h"
#include "../headers/Road.h"
#include "../headers/Map.h"
#include "../headers/Network.h"
#include "../headers/QLearningOperator.h"
#include "../headers/DeepRLOperator.h"
#include "../headers/Intersection.h"
#include "../headers/Vehicle.h"
#include "../headers/Random.h"
#include "../tools/Experiment.h"

// ------------------------- Road tests -------------------------
TEST(RoadTest, BasicBehavior) {
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0.0, 10.0});
    auto r = std::make_unique<Road>(1, i1.get(), i2.get());
    if (r == nullptr) {
        FAIL() << "Failed to create Road instances.";
    }

    std::list<Road*> track{ r.get() };
    auto car = std::make_shared<Car>(i1.get(), i2.get(), 1, i2.get(), track);

    EXPECT_FALSE(r->containVehicle());
    EXPECT_EQ(r->getID(), 1);
    EXPECT_DOUBLE_EQ(r->getLength(), 10.0);
    EXPECT_EQ(r->getStart(), i1.get());
    EXPECT_EQ(r->getEnd(), i2.get());
    EXPECT_EQ(r->getDirection(), (std::array<double,2>{0.0, 1.0}));

    // add / list / remove vehicle
    r->addVehicle(car);
    EXPECT_TRUE(r->containVehicle());
    auto vehicles = r->getVehicles();
    ASSERT_EQ(vehicles.size(), 1u);
    EXPECT_EQ(vehicles.front(), car);

    r->removeVehicle();
    EXPECT_FALSE(r->containVehicle());
}

TEST(RoadTest, VehicleMovement) {
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0.0, 10.0});
    auto r = std::make_unique<Road>(1, i1.get(), i2.get());
    
    std::list<Road*> track; // Empty track means i2 is final destination
    auto car1 = std::make_shared<Car>(i1.get(), i2.get(), 1, i2.get(), track);
    auto car2 = std::make_shared<Car>(i1.get(), i2.get(), 1, i2.get(), track);
    r->addVehicle(car1);
    r->addVehicle(car2);
    EXPECT_TRUE(r->containVehicle());
    
    // Move cars on road
    r->moveVehicles();
    EXPECT_FALSE(car1->getStatus());
    EXPECT_FALSE(car2->getStatus());
}

TEST(RoadTest, CapacityQueueAndDepartures) {
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0.0, 10.0});
    auto r = std::make_unique<Road>(1, i1.get(), i2.get());
    auto car1 = std::make_shared<Car>(i1.get(), i2.get(), 1, i2.get(), std::list<Road*>{});
    auto car2 = std::make_shared<Car>(i1.get(), i2.get(), 2, i2.get(), std::list<Road*>{});

    EXPECT_TRUE(r->canEnter(*car1));
    EXPECT_FALSE(r->isFull());
    r->addVehicle(car1);
    // car1 still sits on the road start: no room for another vehicle
    EXPECT_FALSE(r->canEnter(*car2));
    EXPECT_TRUE(r->isFull());
    EXPECT_EQ(r->getQueueLength(), 1); // Stopped
    // Once car1 moved away by more than the vehicles' spacing, car2 can enter
    car1->setPosition({0.0, 3.0});
    EXPECT_TRUE(r->canEnter(*car2));

    EXPECT_EQ(r->getNumberOfDepartedVehicles(), 0);
    r->removeVehicle();
    EXPECT_EQ(r->getNumberOfDepartedVehicles(), 1);
}

// ------------------------- Map tests -------------------------
TEST(MapTest, Connections) {
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0.0, 10.0});
    auto r = std::make_unique<Road>(1, i1.get(), i2.get());

    Map m;
    m.setConnection(1, 2, r.get());
    m.updateConnection(r);

    EXPECT_EQ(m.getConnection(1, 2), r.get());
    EXPECT_EQ(m.getConnection(2, 1), nullptr);

    auto existing_track = m.track(i1.get(), i2.get());
    ASSERT_EQ(existing_track.size(), 1u);
    EXPECT_EQ(existing_track.front(), r.get());
    
    auto non_existing_track = m.track(i2.get(), i1.get());
    EXPECT_TRUE(non_existing_track.empty());

    auto nothing_track = m.track(i1.get(), i1.get());
    EXPECT_TRUE(nothing_track.empty());
}

// Restores the global simulation parameters modified by a test.
struct ConstantsGuard {
    ConstantsGuard()
        : controllerType(constants::controllerType), boost(constants::boost),
          demand(constants::demand), nbVehicleMax(constants::nbVehicleMax),
          oneWayRoadRatio(constants::oneWayRoadRatio),
          trafficLightPeriod(constants::trafficLightPeriod) {}
    ~ConstantsGuard() {
        constants::controllerType = controllerType;
        constants::boost = boost;
        constants::demand = demand;
        constants::nbVehicleMax = nbVehicleMax;
        constants::oneWayRoadRatio = oneWayRoadRatio;
        constants::trafficLightPeriod = trafficLightPeriod;
    }
    ControllerType controllerType;
    double boost;
    double demand;
    int nbVehicleMax;
    double oneWayRoadRatio;
    double trafficLightPeriod;
};

// ------------------------- Graph tests -------------------------
TEST(GraphTest, StronglyConnected) {
    EXPECT_TRUE(graph::isStronglyConnected(3, {{0, 1}, {1, 2}, {2, 0}}));
    EXPECT_FALSE(graph::isStronglyConnected(3, {{0, 1}, {1, 2}}));
    EXPECT_FALSE(graph::isStronglyConnected(3, {{0, 1}, {1, 0}})); // Node 2 isolated
}

TEST(GraphTest, OrientEdgesRatioExtremes) {
    std::mt19937 rng(42);
    const std::vector<graph::Edge> path = {{0, 1}, {1, 2}, {2, 3}};
    // No one-way road: every edge is two-way.
    EXPECT_EQ(graph::orientEdges(4, path, 0.0, rng).size(), 6u);
    // Only one-way roads requested, but on a path every road must be two-way to connect.
    const auto edges = graph::orientEdges(4, path, 1.0, rng);
    EXPECT_EQ(edges.size(), 6u);
    EXPECT_TRUE(graph::isStronglyConnected(4, edges));
    // On a cycle, all one-way roads can stay one-way only if they all turn the same way.
    const std::vector<graph::Edge> cycle = {{0, 1}, {1, 2}, {2, 3}, {0, 3}};
    for (int seed = 0; seed < 50; ++seed) {
        std::mt19937 r(seed);
        EXPECT_TRUE(graph::isStronglyConnected(4, graph::orientEdges(4, cycle, 1.0, r)));
    }
}

TEST(GraphTest, OrientEdgesKeepsOneWayRoadsAndConnectivity) {
    // Triangulated grid: 4x4 nodes
    std::vector<graph::Edge> mesh;
    auto node = [](int x, int y) { return 4 * y + x; };
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x) {
            if (x < 3) mesh.push_back({node(x, y), node(x + 1, y)});
            if (y < 3) mesh.push_back({node(x, y), node(x, y + 1)});
            if (x < 3 && y < 3) mesh.push_back({node(x, y), node(x + 1, y + 1)});
        }
    size_t totalOneWay = 0;
    for (int seed = 0; seed < 100; ++seed) {
        std::mt19937 rng(seed);
        const auto edges = graph::orientEdges(16, mesh, 0.5, rng);
        ASSERT_TRUE(graph::isStronglyConnected(16, edges)) << "seed " << seed;
        ASSERT_GE(edges.size(), mesh.size());
        ASSERT_LE(edges.size(), 2 * mesh.size());
        totalOneWay += 2 * mesh.size() - edges.size();
    }
    EXPECT_GT(totalOneWay, 0u); // One-way roads are kept when possible
}

TEST(MapTest, LiveCostStaysFiniteWhenRoadStartIsOccupied) {
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0.0, 10.0});
    auto r = std::make_unique<Road>(1, i1.get(), i2.get());

    Map m;
    m.setConnection(1, 2, r.get());
    // A vehicle that just entered sits exactly on the road start (distance 0 to it).
    r->addVehicle(std::make_shared<Car>(i1.get(), i2.get(), 1, i2.get(), std::list<Road*>{}));
    m.updateConnection(r);

    // The road must stay usable (a division by zero used to make it unreachable).
    auto path = Map::track(i1.get(), i2.get());
    ASSERT_EQ(path.size(), 1u);
    EXPECT_EQ(path.front(), r.get());
}

// ------------------------- Network tests -------------------------
// Headless: these run the whole simulation logic without any window / OpenGL context,
// so they are meaningful on every CI platform.
class NetworkHeadlessTest : public ::testing::TestWithParam<ControllerType> {};

TEST_P(NetworkHeadlessTest, SimulationRuns) {
    ConstantsGuard guard;
    constants::controllerType = GetParam();
    constants::demand = 150.0;
    constants::nbVehicleMax = 70;

    Network n(7);
    EXPECT_EQ(n.getIntersectionCount(), constants::nbIntersections);
    EXPECT_GT(n.getRoadCount(), 0);

    const int steps = 60 * 60; // One simulated minute
    for (int k = 0; k < steps; ++k) {
        n.step();
        ASSERT_LE(n.getVehicleCount(), constants::nbVehicleMax);
    }
    EXPECT_NEAR(n.getSimulationTime(), steps * Network::timeStep, 1e-6);
    EXPECT_GT(n.getVehicleCount(), 0);
    EXPECT_GT(n.getCompletedVehicles(), 0); // Some trips must be completed
}

class NetworkRoadsTest : public ::testing::TestWithParam<double> {};

TEST_P(NetworkRoadsTest, PathExistsBetweenAnyTwoIntersections) {
    ConstantsGuard guard;
    constants::oneWayRoadRatio = GetParam();
    for (int attempt = 0; attempt < 10; ++attempt) { // Several random networks
        Network n;
        for (int a = 0; a < n.getIntersectionCount(); ++a) {
            for (int b = 0; b < n.getIntersectionCount(); ++b) {
                if (a == b)
                    continue;
                const auto path = Map::track(n.getIntersection(a), n.getIntersection(b));
                ASSERT_FALSE(path.empty()) << a << " -> " << b;
                EXPECT_EQ(path.front()->getStart()->getID(), a);
                EXPECT_EQ(path.back()->getEnd()->getID(), b);
                for (auto it = path.begin(); std::next(it) != path.end(); ++it) {
                    EXPECT_EQ((*it)->getEnd(), (*std::next(it))->getStart());
                }
            }
        }
    }
}

INSTANTIATE_TEST_SUITE_P(OneWayRatios, NetworkRoadsTest, ::testing::Values(0.0, 0.3, 1.0));

// The heuristic used to keep the green for a vehicle blocked downstream, and the space check used
// pixel sizes as grid units: the network locked up (no more trips) within a few minutes.
TEST(NetworkHeadlessTest, NoGridlock) {
    ConstantsGuard guard;
    const Scenario saturated = *findScenario("saturated");
    for (unsigned int seed = 1; seed <= 3; ++seed) {
        const TrafficStats stats = run(std::make_shared<HeuristicOperator>(), seed, saturated, 10.0, 2.0);
        EXPECT_GT(stats.completed, 100) << "seed " << seed;
    }
}

INSTANTIATE_TEST_SUITE_P(AllControllers, NetworkHeadlessTest,
                         ::testing::Values(ControllerType::RANDOM, ControllerType::FIXED_CYCLE,
                                           ControllerType::HEURISTIC, ControllerType::MAX_PRESSURE,
                                           ControllerType::Q_LEARNING, ControllerType::DQN));

// ------------------------- Demand and statistics tests -------------------------
// The same seed must give the same simulation: experiments are reproducible.
TEST(NetworkHeadlessTest, SameSeedSameResult) {
    ConstantsGuard guard;
    const Scenario medium = *findScenario("medium");
    for (const ControllerType type : {ControllerType::RANDOM, ControllerType::HEURISTIC}) {
        const TrafficStats a = run(makeOperator(type, 3), 3, medium, 1.0, 3.0);
        const TrafficStats b = run(makeOperator(type, 3), 3, medium, 1.0, 3.0);
        EXPECT_EQ(a.completed, b.completed);
        EXPECT_EQ(a.travelTimes, b.travelTimes);
        EXPECT_DOUBLE_EQ(a.timeSpent, b.timeSpent);
    }
    // Learning (exploration, initial weights, replay) is reproducible too
    auto learn = [&]() {
        auto op = std::make_shared<DeepRLOperator>(5);
        return run(op, 3, medium, 0.0, 3.0);
    };
    EXPECT_EQ(learn().travelTimes, learn().travelTimes);
    // Another seed is another network and another demand
    EXPECT_NE(run(makeOperator(ControllerType::HEURISTIC, 3), 3, medium, 1.0, 3.0).travelTimes,
              run(makeOperator(ControllerType::HEURISTIC, 3), 4, medium, 1.0, 3.0).travelTimes);
}

// Controllers are compared under identical traffic conditions: the demand of a seed does not
// depend on what the controller does.
TEST(NetworkHeadlessTest, DemandDoesNotDependOnController) {
    ConstantsGuard guard;
    const Scenario medium = *findScenario("medium");
    const TrafficStats random = run(std::make_shared<RandomOperator>(1), 2, medium, 0.0, 5.0);
    const TrafficStats heuristic = run(std::make_shared<HeuristicOperator>(), 2, medium, 0.0, 5.0);
    EXPECT_EQ(random.generated, heuristic.generated);
    EXPECT_EQ(random.rejected, 0);
    EXPECT_EQ(heuristic.rejected, 0);
    // Poisson arrivals: 120 vehicles/min during 5 min = 600 +- a few standard deviations (24)
    EXPECT_NEAR(random.generated, 600, 100);
    // ...but not the traffic: the heuristic loses less time
    EXPECT_LT(heuristic.timeSpent, random.timeSpent);
}

TEST(NetworkHeadlessTest, StatisticsAreConsistent) {
    ConstantsGuard guard;
    constants::controllerType = ControllerType::HEURISTIC;
    constants::demand = 200.0;
    constants::nbVehicleMax = 60;
    Network n(11);
    const int steps = 5 * 60 * 60; // 5 simulated minutes
    for (int k = 0; k < steps; ++k)
        n.step();
    const TrafficStats& s = n.getStats();
    EXPECT_NEAR(s.duration, 300.0, 1e-6);
    // Every generated vehicle is somewhere
    EXPECT_EQ(s.generated, s.rejected + s.entered + n.getWaitingVehicleCount());
    EXPECT_EQ(s.entered, s.completed + n.getVehicleCount());
    EXPECT_EQ(s.completed, n.getCompletedVehicles());
    EXPECT_EQ(static_cast<int>(s.travelTimes.size()), s.completed);
    EXPECT_NEAR(s.flow(), s.completed / 5.0, 1e-6);
    // Waiting is part of the time in the network, itself part of the time spent
    EXPECT_GT(s.timeWaiting, 0.0);
    EXPECT_LT(s.timeWaiting, s.timeInNetwork);
    EXPECT_LE(s.timeInNetwork, s.timeSpent);
    EXPECT_LE(s.meanVehicles(), 60.0);
    EXPECT_GT(s.meanRelativeSpeed(), 0.0);
    EXPECT_LT(s.meanRelativeSpeed(), 1.0);
    // Traffic lights and traffic make trips longer than on an empty network
    EXPECT_GT(s.meanDelay(), 0.0);
    EXPECT_LT(s.meanDelay(), s.meanTravelTime());
    EXPECT_LE(s.meanTravelTime(), s.travelTimePercentile(1.0));
    EXPECT_LE(s.travelTimePercentile(0.5), s.travelTimePercentile(0.95));

    n.resetStats();
    EXPECT_EQ(n.getStats().completed, 0);
    EXPECT_EQ(n.getStats().duration, 0.0);
    EXPECT_EQ(TrafficStats().meanTravelTime(), 0.0); // No division by zero without data
    EXPECT_EQ(TrafficStats().timeSpentPerVehicle(), 0.0);
}

TEST(NetworkHeadlessTest, NoDemandNoVehicle) {
    ConstantsGuard guard;
    constants::demand = 0.0;
    Network n(1);
    for (int k = 0; k < 600; ++k)
        n.step();
    EXPECT_EQ(n.getVehicleCount(), 0);
    EXPECT_EQ(n.getStats().generated, 0);
}

// Vehicles stay in order on a road: none goes through the vehicle in front.
TEST(NetworkHeadlessTest, VehiclesNeverOvertake) {
    ConstantsGuard guard;
    constants::controllerType = ControllerType::FIXED_CYCLE;
    constants::demand = 300.0;
    constants::nbVehicleMax = 150;
    Network n(4);
    for (int k = 0; k < 3 * 60 * 60; ++k) {
        n.step();
        for (const auto& r : n.getRoads()) {
            double previous = -1.0;
            for (const auto& v : r->getVehicles()) {
                const std::vector<double>& p = v->getPosition();
                const std::vector<double>& end = r->getEnd()->getPosition();
                const double toEnd = std::hypot(p[0] - end[0], p[1] - end[1]);
                ASSERT_GE(toEnd, previous) << "step " << k << ", road " << r->getID();
                previous = toEnd;
            }
        }
    }
}

// ------------------------- Controller performance tests -------------------------
// The purpose of the project: the trained controllers must manage traffic better than the
// baselines, on networks they were not trained on.
TEST(ControllerPerformanceTest, TrainedControllersBeatBaselines) {
    ConstantsGuard guard;
    auto dqn = std::make_shared<DeepRLOperator>();
    auto qlearning = std::make_shared<QLearningOperator>();
    if (!dqn->load(DeepRLOperator::defaultModelPath) || !qlearning->load(QLearningOperator::defaultModelPath)) {
        GTEST_SKIP() << "No trained model in the working directory (run the train tool)";
    }
    dqn->setTraining(false);
    qlearning->setTraining(false);
    for (const char* name : {"light", "heavy", "saturated"}) {
        const Scenario scenario = *findScenario(name);
        for (unsigned int seed = 21; seed <= 23; ++seed) {
            auto timeSpent = [&](const std::shared_ptr<IntersectionOperator>& op) {
                return run(op, seed, scenario, 2.0, 6.0).timeSpentPerVehicle();
            };
            const double random = timeSpent(std::make_shared<RandomOperator>(seed));
            const double fixed = timeSpent(std::make_shared<FixedCycleOperator>());
            const double heuristic = timeSpent(std::make_shared<HeuristicOperator>());
            EXPECT_LT(fixed, random) << name << ", seed " << seed;
            EXPECT_LT(heuristic, fixed) << name << ", seed " << seed;
            for (const auto& [learner, time] : {std::pair{"dqn", timeSpent(dqn)}, std::pair{"qlearning", timeSpent(qlearning)}}) {
                EXPECT_LT(time, 0.8 * random) << learner << ", " << name << ", seed " << seed;
                EXPECT_LT(time, 0.95 * fixed) << learner << ", " << name << ", seed " << seed;
                EXPECT_LT(time, 1.05 * heuristic) << learner << ", " << name << ", seed " << seed;
            }
        }
    }
}

// Learning from scratch works: a short training is enough to beat the fixed cycle.
TEST(ControllerPerformanceTest, LearningFromScratchBeatsFixedCycle) {
    ConstantsGuard guard;
    const Scenario medium = *findScenario("medium");
    auto dqn = std::make_shared<DeepRLOperator>(1);
    dqn->setExploration(1.0, 0.05, 0.999);
    for (unsigned int seed = 100; seed < 106; ++seed)
        run(dqn, seed, medium, 0.0, 10.0);
    dqn->setTraining(false);
    for (unsigned int seed = 31; seed <= 32; ++seed) {
        const double fixed = run(std::make_shared<FixedCycleOperator>(), seed, medium, 2.0, 6.0).timeSpentPerVehicle();
        EXPECT_LT(run(dqn, seed, medium, 2.0, 6.0).timeSpentPerVehicle(), fixed) << "seed " << seed;
    }
}

TEST(NetworkTest, TerminalOutput) {
    Network n(1);
    testing::internal::CaptureStdout();
    n.printTerminalOutput(12.5, 3.0, 60, 1.0);
    const std::string summary = testing::internal::GetCapturedStdout();
    EXPECT_NE(summary.find("Flow/min: 12.5"), std::string::npos);
    EXPECT_EQ(summary.find("Control Panel"), std::string::npos);

    n.setTerminalOutput(true);
    testing::internal::CaptureStdout();
    n.printTerminalOutput(12.5, 3.0, 60, 1.0);
    const std::string panel = testing::internal::GetCapturedStdout();
    for (const char* field : {"Control Panel", "FPS: 60", "Flow: 12.5", "Boost:", "Method:", "State: PLAYING"}) {
        EXPECT_NE(panel.find(field), std::string::npos) << field;
    }
}

// These open a window: they exercise the rendering path when a display is available
// and return immediately otherwise.
TEST(NetworkTest, SimulationSteps_QLearning) {
    ConstantsGuard guard;
    constants::controllerType = ControllerType::Q_LEARNING;
    constants::boost = 100.0;
    constants::demand = 600.0; // Many vehicles to hit move/intersection logic

    Network n;
    n.displayNetwork(10); 
    SUCCEED();
}

TEST(NetworkTest, SimulationSteps_DQN) {
    ConstantsGuard guard;
    constants::controllerType = ControllerType::DQN;
    constants::boost = 100.0;
    constants::demand = 600.0;

    Network n;
    n.displayNetwork(10);
    SUCCEED();
}

TEST(NetworkTest, SimulationSteps_Heuristic) {
    ConstantsGuard guard;
    constants::controllerType = ControllerType::HEURISTIC;
    constants::boost = 100.0;
    constants::demand = 600.0;

    Network n;
    n.displayNetwork(10);
    SUCCEED();
}

// ------------------------- Vehicle tests -------------------------
TEST(VehicleTest, StopsAtRedLightThenCrossesOnGreen) {
    auto i0 = std::make_unique<Intersection>(0, std::vector<double>{10.0, 10.0});
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0.0, 10.0},
                                             std::make_shared<HeuristicOperator>());
    auto other = std::make_unique<Road>(99, i0.get(), i2.get());
    auto r = std::make_unique<Road>(1, i1.get(), i2.get());
    i2->addInputRoad(other.get()); // Index 0: green at start
    i2->addInputRoad(r.get());     // Index 1: red

    auto car = std::make_shared<Car>(i1.get(), i2.get(), 1, i2.get(), std::list<Road*>{});
    r->addVehicle(car);
    ASSERT_TRUE(i2->isRed(r->getID()));

    for (int k = 0; k < 500; ++k) {
        r->moveVehicles();
    }
    EXPECT_FALSE(car->getStatus()) << "The vehicle ran the red light";
    EXPECT_TRUE(r->containVehicle());
    EXPECT_GT(car->distance(i2.get()), 0.0);
    // The vehicle is waiting at the light, and the time it waits is measured
    EXPECT_TRUE(car->isWaiting());
    EXPECT_EQ(r->getQueueLength(), 1);
    EXPECT_GT(car->getWaitingTime(), 5.0);
    EXPECT_LT(car->getWaitingTime(), 500 * constants::timeStep);
    EXPECT_DOUBLE_EQ(r->getHeadWaitingTime(), car->getRoadWaitingTime());

    // The heuristic gives the green to the only road with a vehicle.
    i2->update(constants::trafficLightPeriod * 2.0);
    ASSERT_FALSE(i2->isRed(r->getID()));
    for (int k = 0; k < 500 && !car->getStatus(); ++k) {
        r->moveVehicles();
    }
    EXPECT_TRUE(car->getStatus());
    EXPECT_FALSE(r->containVehicle());
}

TEST(VehicleTest, MovementLogic) {
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0.0, 100.0});
    auto r = std::make_unique<Road>(1, i1.get(), i2.get());
    std::list<Road*> track;

    Car car(i1.get(), i2.get(), 1, i2.get(), track);
    
    // Test breaking speed
    EXPECT_GT(car.breakingSpeed(10.0), 0.0);
    EXPECT_LT(car.breakingSpeed(1.0), car.getSpeedMax());

    // Test moving to intersection (Green)
    car.moveToIntersection(i2.get(), 1);
    EXPECT_GT(car.getSpeed(), 0.0);
    EXPECT_GT(car.getPosition()[1], 0.0);

    // Test moving to vehicle (follow leader)
    auto leader = std::make_shared<Car>(i1.get(), i2.get(), 2, i2.get(), track);
    leader->setPosition({0.0, 50.0});
    
    car.setPosition({0.0, 45.0});
    car.moveToVehicle(leader);
    EXPECT_LT(car.getPosition()[1], 50.0);
}

TEST(VehicleTest, Properties) {
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0,0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0,10});
    std::list<Road*> track;

    Bike b(i1.get(), i2.get(), 1, i2.get(), track);
    Car c(i1.get(), i2.get(), 2, i2.get(), track);
    Truck t(i1.get(), i2.get(), 3, i2.get(), track);

    // ID
    EXPECT_EQ(b.getID(), 1);

    // Destination
    EXPECT_EQ(c.getDestination(), i2.get());
    EXPECT_EQ(c.nextRoad(), nullptr);

    // Trip
    t.setTrip(12.0, 34.0);
    EXPECT_DOUBLE_EQ(t.getCreationTime(), 12.0);
    EXPECT_DOUBLE_EQ(t.getFreeFlowTime(), 34.0);
    EXPECT_DOUBLE_EQ(t.getWaitingTime(), 0.0);

    // Speed: Truck < Car < Bike
    EXPECT_GT(b.getSpeedMax(), c.getSpeedMax());
    EXPECT_GT(c.getSpeedMax(), t.getSpeedMax());

    // Acceleration: Truck < Car < Bike
    EXPECT_GT(b.getAcceleration(), c.getAcceleration());
    EXPECT_GT(c.getAcceleration(), t.getAcceleration());
    
    // Width: Bike < Car < Truck
    EXPECT_LT(b.getWidth(), c.getWidth());
    EXPECT_LT(c.getWidth(), t.getWidth());

    // Length: Bike < Car < Truck
    EXPECT_LT(b.getHeight(), c.getHeight());
    EXPECT_LT(c.getHeight(), t.getHeight());

    // Color: Bike != Car != Truck
    EXPECT_NE(b.getColor(), c.getColor());
    EXPECT_NE(c.getColor(), t.getColor());
    EXPECT_NE(b.getColor(), t.getColor());

    // Type
    EXPECT_TRUE(b.is2Wheeler());
    EXPECT_FALSE(c.is2Wheeler());
    EXPECT_FALSE(t.is2Wheeler());
}

// ------------------------- Intersection tests -------------------------
TEST(IntersectionTest, Basic) {
    auto op = std::make_shared<QLearningOperator>();
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{10.0, 20.0}, op);
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{10.0, 30.0}, op);
    auto i3 = std::make_unique<Intersection>(3, std::vector<double>{10.0, 40.0}, op);
    if (i1 == nullptr || i2 == nullptr || i3 == nullptr) {
        FAIL() << "Failed to create Intersection instances.";
    }

    auto r1 = std::make_unique<Road>(1, i1.get(), i3.get());
    auto r2 = std::make_unique<Road>(2, i2.get(), i3.get());

    i3->addInputRoad(r1.get());
    i3->addInputRoad(r2.get());

    EXPECT_EQ(i1->getID(), 1);
    EXPECT_EQ(i1->getPosition()[0], 10.0);
    EXPECT_EQ(i1->getPosition()[1], 20.0);
    EXPECT_TRUE(*i1 == *i1);
    EXPECT_FALSE(*i1 == *i2);
    
    // Initially green
    EXPECT_FALSE(i3->isRed(1));
    
    EXPECT_EQ(i3->getNumberInputRoads(), 2);
    
    // Test that it doesn't crash during update
    for (int k=0; k<10; ++k) {
        i3->update(k * constants::trafficLightPeriod);
    }
    SUCCEED();
}

// A vehicle on one of the 2 input roads of an intersection: what the controllers observe.
TEST(IntersectionTest, ObservationAndReward) {
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{20.0, 20.0});
    auto i3 = std::make_unique<Intersection>(3, std::vector<double>{0.0, 20.0});
    auto r1 = std::make_unique<Road>(1, i1.get(), i3.get());
    auto r2 = std::make_unique<Road>(2, i2.get(), i3.get());
    i3->addInputRoad(r1.get()); // Index 0: green
    i3->addInputRoad(r2.get()); // Index 1: red
    auto car = std::make_shared<Car>(i2.get(), i3.get(), 1, i3.get(), std::list<Road*>{});
    r2->addVehicle(car);
    for (int k = 0; k < 600; ++k) // 10 s: the car reaches the red light and waits
        r2->moveVehicles();

    const Observation o = i3->observe(10.0);
    ASSERT_EQ(o.approaches.size(), 2u);
    EXPECT_EQ(o.currentGreen, 0);
    EXPECT_EQ(i3->getGreenRoadIndex(), 0);
    EXPECT_DOUBLE_EQ(o.greenDuration, 10.0);
    // Empty road
    EXPECT_EQ(o.approaches[0].vehicles, 0);
    EXPECT_EQ(o.approaches[0].queue, 0);
    EXPECT_FALSE(o.approaches[0].canDischarge);
    EXPECT_DOUBLE_EQ(o.approaches[0].headDistance, 20.0);
    EXPECT_DOUBLE_EQ(o.approaches[0].length, 20.0);
    // Road with the waiting car
    EXPECT_EQ(o.approaches[1].vehicles, 1);
    EXPECT_EQ(o.approaches[1].queue, 1);
    EXPECT_TRUE(o.approaches[1].canDischarge);
    EXPECT_FALSE(o.approaches[1].full);
    EXPECT_GT(o.approaches[1].headWaitingTime, 5.0);
    EXPECT_LT(o.approaches[1].headDistance, 1.0);
    EXPECT_DOUBLE_EQ(o.approaches[1].downstreamVehicles, 0.0);

    // Reward: fewer vehicles waiting, more departures and shorter waits are better
    EXPECT_DOUBLE_EQ(Intersection::reward(0.0, 0, 0.0), 0.0);
    EXPECT_LT(Intersection::reward(5.0, 0, 0.0), Intersection::reward(2.0, 0, 0.0));
    EXPECT_GT(Intersection::reward(2.0, 3, 0.0), Intersection::reward(2.0, 0, 0.0));
    EXPECT_LT(Intersection::reward(2.0, 0, 120.0), Intersection::reward(2.0, 0, 20.0));
    // A departure never pays for a vehicle left waiting: no reward for moving vehicles around
    EXPECT_LT(Intersection::reward(1.0, 1, 0.0), 0.0);
}

// ------------------------- Controller tests -------------------------
namespace {
    /** Observation of an intersection with the given number of vehicles (all waiting) per road. */
    Observation makeObservation(const std::vector<int>& vehicles, const int green = 0) {
        Observation o;
        o.currentGreen = green;
        o.greenDuration = 2.0;
        for (const int n : vehicles) {
            ApproachObservation a;
            a.vehicles = n;
            a.queue = n;
            a.canDischarge = n > 0;
            a.headDistance = n > 0 ? 0.3 : 30.0;
            a.headWaitingTime = n > 0 ? 4.0 : 0.0;
            a.length = 30.0;
            o.approaches.push_back(a);
        }
        return o;
    }
}

TEST(ControllerTest, Baselines) {
    const Observation o = makeObservation({1, 0, 5}, 2);
    // Fixed cycle: next road, whatever the traffic
    FixedCycleOperator fixed;
    EXPECT_EQ(fixed.decide(o), 0);
    EXPECT_EQ(fixed.decide(makeObservation({1, 0, 5}, 0)), 1);
    FixedCycleOperator slow(6.0); // Green for 6 s: kept after 2 s
    EXPECT_EQ(slow.decide(o), 2);
    EXPECT_FALSE(fixed.isLearning());
    // Random: every road, about as often
    RandomOperator random(1);
    std::vector<int> count(3, 0);
    for (int k = 0; k < 3000; ++k)
        count[random.decide(o)]++;
    for (const int c : count)
        EXPECT_NEAR(c, 1000, 150);
}

TEST(ControllerTest, HeuristicAndMaxPressure) {
    HeuristicOperator heuristic;
    MaxPressureOperator maxPressure;
    // The green road is empty: serve the busiest road
    EXPECT_EQ(heuristic.decide(makeObservation({0, 2, 5})), 2);
    EXPECT_EQ(maxPressure.decide(makeObservation({0, 2, 5})), 2);
    // Traffic is going through the green road: the heuristic keeps it
    EXPECT_EQ(heuristic.decide(makeObservation({1, 2, 5})), 0);
    // A road whose first vehicle is blocked downstream is not served
    Observation blocked = makeObservation({0, 2, 5});
    blocked.approaches[2].canDischarge = false;
    EXPECT_EQ(heuristic.decide(blocked), 1);
    EXPECT_EQ(maxPressure.decide(blocked), 1);
    // A full road first (its queue reaches the previous intersection)
    Observation full = makeObservation({0, 2, 5});
    full.approaches[1].full = true;
    EXPECT_EQ(heuristic.decide(full), 1);
    // Max pressure: vehicles going to a busy road count less
    Observation pressure = makeObservation({0, 4, 5});
    pressure.approaches[2].downstreamVehicles = 4.0;
    EXPECT_EQ(maxPressure.decide(pressure), 1);
    // Nothing to do: keep the green
    EXPECT_EQ(heuristic.decide(makeObservation({0, 0, 0}, 1)), 1);
    EXPECT_EQ(maxPressure.decide(makeObservation({0, 0, 0}, 1)), 1);
}

TEST(ControllerTest, Factory) {
    for (const ControllerType type :
         {ControllerType::RANDOM, ControllerType::FIXED_CYCLE, ControllerType::HEURISTIC,
          ControllerType::MAX_PRESSURE, ControllerType::Q_LEARNING, ControllerType::DQN}) {
        const auto op = makeOperator(type, 1);
        ASSERT_NE(op, nullptr);
        const int action = op->decide(makeObservation({1, 2, 3}));
        EXPECT_GE(action, 0);
        EXPECT_LT(action, 3);
        ControllerType parsed;
        ASSERT_TRUE(parseController(controllerName(type), parsed));
        EXPECT_EQ(parsed, type);
    }
}

// ------------------------- Features tests -------------------------
TEST(FeaturesTest, DescribeDecision) {
    const Observation o = makeObservation({1, 4, 2}, 0);
    for (int action = 0; action < 3; ++action) {
        const std::vector<double> f = features::describe(o, action);
        ASSERT_EQ(static_cast<int>(f.size()), features::count);
        for (const double x : f) {
            EXPECT_GE(x, 0.0);
            EXPECT_LE(x, 3.0);
        }
        EXPECT_EQ(f[0], action == 0 ? 1.0 : 0.0); // Is it the green road?
        const int key = features::discretize(o, action);
        EXPECT_GE(key, 0);
        EXPECT_LT(key, features::tableSize);
    }
    // Different decisions are told apart
    EXPECT_NE(features::describe(o, 1), features::describe(o, 2));
    EXPECT_NE(features::discretize(o, 0), features::discretize(o, 1));
    // The description does not depend on the order of the other roads: one model for all
    // the intersections
    const Observation swapped = makeObservation({1, 2, 4}, 0);
    EXPECT_EQ(features::describe(o, 1), features::describe(swapped, 2));
    EXPECT_EQ(features::discretize(o, 1), features::discretize(swapped, 2));
    EXPECT_EQ(features::describe(o, 0), features::describe(swapped, 0));
    // ...nor on the number of roads, when the added road is empty
    const Observation larger = makeObservation({1, 4, 2, 0}, 0);
    EXPECT_EQ(features::discretize(o, 1), features::discretize(larger, 1));
    // Extreme values stay bounded
    Observation extreme = makeObservation({1000, 1000}, 0);
    extreme.approaches[1].headWaitingTime = 1e6;
    extreme.greenDuration = 1e6;
    for (const double x : features::describe(extreme, 1))
        EXPECT_LE(x, 3.0);
    EXPECT_LT(features::discretize(extreme, 1), features::tableSize);
}

// ------------------------- Learning controller tests -------------------------
namespace {
    /** Teach a controller that serving the busiest road is rewarded, then check what it does. */
    template <typename Operator>
    void expectLearnsToServeBusiestRoad() {
        Operator op(1);
        op.setExploration(1.0, 0.1, 0.999);
        EXPECT_TRUE(op.isLearning());
        std::mt19937 rng(1);
        for (int k = 0; k < 6000; ++k) {
            const Observation o = makeObservation({rnd::index(rng, 6), rnd::index(rng, 6)}, rnd::index(rng, 2));
            const Observation next = makeObservation({rnd::index(rng, 6), rnd::index(rng, 6)}, rnd::index(rng, 2));
            const int action = op.decide(o);
            const int other = 1 - action;
            // Reward: minus the vehicles left waiting on the road that is not served
            op.learn(o, action, -o.approaches[other].vehicles / 5.0, next);
        }
        EXPECT_LT(op.getEpsilon(), 0.2);
        op.setTraining(false);
        EXPECT_FALSE(op.isLearning());
        EXPECT_EQ(op.decide(makeObservation({5, 0}, 0)), 0);
        EXPECT_EQ(op.decide(makeObservation({5, 0}, 1)), 0);
        EXPECT_EQ(op.decide(makeObservation({0, 5}, 0)), 1);
        EXPECT_EQ(op.decide(makeObservation({0, 5}, 1)), 1);

        // A saved model gives the same decisions once loaded
        const std::string path = "test_model.tmp";
        ASSERT_TRUE(op.save(path));
        Operator loaded(2);
        ASSERT_TRUE(loaded.load(path));
        loaded.setTraining(false);
        for (int k = 0; k < 50; ++k) {
            const Observation o = makeObservation({rnd::index(rng, 6), rnd::index(rng, 6), rnd::index(rng, 6)}, rnd::index(rng, 3));
            EXPECT_EQ(loaded.decide(o), op.decide(o));
        }
        std::remove(path.c_str());
        // Missing or invalid files are refused, and the model is kept
        EXPECT_FALSE(loaded.load("does_not_exist.tmp"));
        { std::ofstream(path) << "not a model"; }
        EXPECT_FALSE(loaded.load(path));
        std::remove(path.c_str());
        EXPECT_EQ(loaded.decide(makeObservation({5, 0}, 1)), 0);
    }
}

TEST(LearningControllerTest, QLearningLearns) {
    expectLearnsToServeBusiestRoad<QLearningOperator>();
}

TEST(LearningControllerTest, DQNLearns) {
    expectLearnsToServeBusiestRoad<DeepRLOperator>();
}

// Without training, a controller only applies what it knows: no exploration, no update.
TEST(LearningControllerTest, NoTrainingNoChange) {
    DeepRLOperator op(1);
    op.setTraining(false);
    const Observation o = makeObservation({3, 1, 2});
    const int action = op.decide(o);
    for (int k = 0; k < 1000; ++k) {
        op.learn(o, 1, -100.0, o);
        EXPECT_EQ(op.decide(o), action);
    }
}

// ------------------------- Neural network tests -------------------------
TEST(NeuralNetworkTest, LearnsAFunction) {
    NeuralNetwork nn({2, 16, 16, 1}, 0.01, 1);
    EXPECT_EQ(nn.getTopology(), (std::vector<int>{2, 16, 16, 1}));
    EXPECT_EQ(nn.predict({1.0, 0.5}).size(), 1u);
    // XOR: not linear, so the hidden layers are needed
    const std::vector<std::pair<std::vector<double>, double>> samples = {
        {{0.0, 0.0}, 0.0}, {{0.0, 1.0}, 1.0}, {{1.0, 0.0}, 1.0}, {{1.0, 1.0}, 0.0}};
    for (int epoch = 0; epoch < 2000; ++epoch) {
        for (const auto& [input, target] : samples)
            nn.accumulateGradient(input, {target});
        nn.applyGradients();
    }
    for (const auto& [input, target] : samples)
        EXPECT_NEAR(nn.predict(input)[0], target, 0.1);
}

TEST(NeuralNetworkTest, GradientMatchesFiniteDifferences) {
    // One training step with a small learning rate must reduce the error on the sample
    NeuralNetwork nn({3, 8, 2}, 0.001, 3);
    const std::vector<double> input = {0.2, -0.4, 0.9}, target = {0.5, -0.5};
    auto error = [&]() {
        const std::vector<double> out = nn.predict(input);
        return std::pow(out[0] - target[0], 2) + std::pow(out[1] - target[1], 2);
    };
    double previous = error();
    for (int k = 0; k < 20; ++k) {
        nn.train(input, target);
        const double current = error();
        EXPECT_LT(current, previous);
        previous = current;
    }
}

TEST(NeuralNetworkTest, SameSeedSameWeights) {
    const NeuralNetwork a({4, 8, 1}, 0.01, 7), b({4, 8, 1}, 0.01, 7), c({4, 8, 1}, 0.01, 8);
    const std::vector<double> input = {0.1, 0.2, 0.3, 0.4};
    EXPECT_EQ(a.predict(input), b.predict(input));
    EXPECT_NE(a.predict(input), c.predict(input));
}

TEST(NeuralNetworkTest, SaveAndLoad) {
    NeuralNetwork nn({3, 5, 2}, 0.01, 1);
    std::stringstream stream;
    nn.save(stream);
    NeuralNetwork loaded;
    ASSERT_TRUE(loaded.load(stream));
    const std::vector<double> input = {0.3, -1.2, 2.0};
    EXPECT_EQ(loaded.predict(input), nn.predict(input));
    // Truncated or invalid data is refused, and the network is kept
    std::stringstream truncated("3 3 5 2\n0.1 0.2");
    EXPECT_FALSE(loaded.load(truncated));
    std::stringstream invalid("2 3 -1\n");
    EXPECT_FALSE(loaded.load(invalid));
    EXPECT_EQ(loaded.predict(input), nn.predict(input));
}

// ------------------------- Regression tests of the overload fixes -------------------------
// A vehicle is not sent back where it comes from: on a congested two-way road, vehicles doing
// U-turns in both directions block each other for ever.
TEST(MapTest, NoUTurnUnlessDeadEnd) {
    auto a = std::make_unique<Intersection>(0, std::vector<double>{0.0, 0.0});
    auto b = std::make_unique<Intersection>(1, std::vector<double>{10.0, 0.0});
    auto c = std::make_unique<Intersection>(2, std::vector<double>{10.0, 10.0});
    auto ab = std::make_unique<Road>(0, a.get(), b.get());
    auto ba = std::make_unique<Road>(1, b.get(), a.get());
    auto bc = std::make_unique<Road>(2, b.get(), c.get());
    auto ca = std::make_unique<Road>(3, c.get(), a.get());
    Map m;
    m.setConnection(0, 1, ab.get());
    m.setConnection(1, 0, ba.get());
    m.setConnection(1, 2, bc.get());
    m.setConnection(2, 0, ca.get());

    // From b to a: the direct road is the shortest...
    EXPECT_EQ(Map::track(b.get(), a.get()), (std::list<Road*>{ba.get()}));
    // ...but not for a vehicle arriving by the road a -> b, which goes around
    EXPECT_EQ(Map::track(b.get(), a.get(), ab.get()), (std::list<Road*>{bc.get(), ca.get()}));
    // The road the vehicle arrives by only matters at its end
    EXPECT_EQ(Map::track(b.get(), a.get(), ca.get()), (std::list<Road*>{ba.get()}));
    // Dead end: the U-turn is the only way
    Map deadEnd;
    deadEnd.setConnection(0, 1, ab.get());
    deadEnd.setConnection(1, 0, ba.get());
    EXPECT_EQ(Map::track(b.get(), a.get(), ab.get()), (std::list<Road*>{ba.get()}));
}

// The learning controllers never consider a road whose vehicles cannot move while another
// road can be served, and serve first the roads filled by their queue.
TEST(FeaturesTest, UsefulActions) {
    // Roads with a vehicle that can leave, the green one first
    EXPECT_EQ(features::usefulActions(makeObservation({2, 0, 3}, 2)), (std::vector<int>{2, 0}));
    EXPECT_EQ(features::usefulActions(makeObservation({2, 0, 3}, 1)), (std::vector<int>{0, 2}));
    // Blocked road: not a candidate
    Observation blocked = makeObservation({2, 4, 3}, 1);
    blocked.approaches[1].canDischarge = false;
    EXPECT_EQ(features::usefulActions(blocked), (std::vector<int>{0, 2}));
    // Full roads first, if they can be served
    Observation full = makeObservation({2, 4, 3}, 0);
    full.approaches[2].full = true;
    EXPECT_EQ(features::usefulActions(full), (std::vector<int>{2}));
    full.approaches[2].canDischarge = false;
    EXPECT_EQ(features::usefulActions(full), (std::vector<int>{0, 1}));
    // Nothing can move: keep the green
    EXPECT_EQ(features::usefulActions(makeObservation({0, 0, 0}, 1)), (std::vector<int>{1}));

    // Whatever they learned, and even while exploring, the controllers respect it
    QLearningOperator qlearning(1);
    DeepRLOperator dqn(1);
    for (int k = 0; k < 200; ++k) {
        EXPECT_NE(qlearning.decide(blocked), 1);
        EXPECT_NE(dqn.decide(blocked), 1);
    }
}

// The overload that used to freeze the network with the learning controllers on this seed
// (green light held on a blocked road, then U-turn deadlock).
TEST(ControllerPerformanceTest, NoFreezeUnderOverload) {
    ConstantsGuard guard;
    auto dqn = std::make_shared<DeepRLOperator>();
    if (!dqn->load(DeepRLOperator::defaultModelPath)) {
        GTEST_SKIP() << "No trained model in the working directory (run the train tool)";
    }
    dqn->setTraining(false);
    const TrafficStats stats = run(dqn, 17, Scenario{"overload", 600.0, 100}, 8.0, 2.0);
    EXPECT_GT(stats.completed, 200);
}
