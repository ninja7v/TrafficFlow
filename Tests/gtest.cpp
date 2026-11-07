// Librairies
#include <gtest/gtest.h>  // for Google Test framework
#include <memory>         // for std::make_unique and std::make_shared
#include <list>           // for std::list
#include <array>          // for std::array
#include <vector>         // for std::vector
// Headers
#include "../headers/Bike.h"
#include "../headers/Car.h"
#include "../headers/Truck.h"
#include "../headers/Constants.h"
#include "../headers/Graph.h"
#include "../headers/Road.h"
#include "../headers/Map.h"
#include "../headers/Network.h"
#include "../headers/QLearningOperator.h"
#include "../headers/DeepRLOperator.h"
#include "../headers/Intersection.h"
#include "../headers/Vehicle.h"

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
    EXPECT_EQ(r->getTotalNumberOfArringVehicles(), 1);
    auto stats = r->getVehicleStats(1.0);
    EXPECT_EQ(std::get<0>(stats), 1); // Occupancy: Low
    EXPECT_EQ(std::get<1>(stats), 0); // Speed: Slow
    EXPECT_EQ(std::get<2>(stats), 1); // Usage: Medium
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
        : learningType(constants::learningType), boost(constants::boost),
          flow(constants::flow), nbVehicleMax(constants::nbVehicleMax),
          oneWayRoadRatio(constants::oneWayRoadRatio) {}
    ~ConstantsGuard() {
        constants::learningType = learningType;
        constants::boost = boost;
        constants::updateBoostDependentConstants();
        constants::flow = flow;
        constants::nbVehicleMax = nbVehicleMax;
        constants::oneWayRoadRatio = oneWayRoadRatio;
    }
    LearningType learningType;
    double boost;
    int flow;
    int nbVehicleMax;
    double oneWayRoadRatio;
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
class NetworkHeadlessTest : public ::testing::TestWithParam<LearningType> {};

TEST_P(NetworkHeadlessTest, SimulationRuns) {
    ConstantsGuard guard;
    constants::learningType = GetParam();
    constants::boost = 1.0;
    constants::updateBoostDependentConstants();
    constants::nbVehicleMax = 70;

    Network n;
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

INSTANTIATE_TEST_SUITE_P(AllOperators, NetworkHeadlessTest,
                         ::testing::Values(LearningType::Q_LEARNING, LearningType::DQN,
                                           LearningType::HEURISTIC));

// These open a window: they exercise the rendering path when a display is available
// and return immediately otherwise.
TEST(NetworkTest, SimulationSteps_QLearning) {
    ConstantsGuard guard;
    constants::learningType = LearningType::Q_LEARNING;
    constants::boost = 100.0;
    constants::updateBoostDependentConstants();
    constants::flow = 99; // Maximize vehicle spawns to hit move/intersection logic

    Network n;
    n.displayNetwork(10); 
    SUCCEED();
}

TEST(NetworkTest, SimulationSteps_DQN) {
    ConstantsGuard guard;
    constants::learningType = LearningType::DQN;
    constants::boost = 100.0;
    constants::updateBoostDependentConstants();
    constants::flow = 99;

    Network n;
    n.displayNetwork(10);
    SUCCEED();
}

TEST(NetworkTest, SimulationSteps_Heuristic) {
    ConstantsGuard guard;
    constants::learningType = LearningType::HEURISTIC;
    constants::boost = 100.0;
    constants::updateBoostDependentConstants();
    constants::flow = 99;

    Network n;
    n.displayNetwork(10);
    SUCCEED();
}

// ------------------------- Vehicle tests -------------------------
TEST(VehicleTest, StopsAtRedLightThenCrossesOnGreen) {
    ConstantsGuard guard;
    constants::learningType = LearningType::HEURISTIC;
    constants::boost = 100.0; // Very fast: one step is longer than the road
    constants::updateBoostDependentConstants();

    auto i0 = std::make_unique<Intersection>(0, std::vector<double>{10.0, 10.0});
    auto i1 = std::make_unique<Intersection>(1, std::vector<double>{0.0, 0.0});
    auto i2 = std::make_unique<Intersection>(2, std::vector<double>{0.0, 10.0});
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

    // enterRoadTime
    EXPECT_GT(t.getEnterRoadTime(), 0);

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

// ------------------------- Intersection Operator tests -------------------------
TEST(IntersectionOperatorTest, Basic) {
    auto op1 = std::make_unique<QLearningOperator>();
    auto op2 = std::make_unique<DeepRLOperator>();
    if (op1 == nullptr || op2 == nullptr) {
        FAIL() << "Failed to create IntersectionOperator instances.";
    }

    const std::vector<int> state = {1, 0, 1, 0, 1, 0, 1}; // size 7
    const std::vector<int> actions = {0, 1};
    const double reward = 1.0;

    int action1 = op1->decide(state, actions);
    int action2 = op2->decide(state, actions);

    op1->learn(state, action1, reward, state, actions);
    op2->learn(state, action2, reward, state, actions);

    // Same state
    action1 = op1->decide(state, actions);
    action2 = op2->decide(state, actions);

    op1->learn(state, action1, reward, state, actions);
    op2->learn(state, action2, reward, state, actions);
    
    // New state
    std::vector<int> nextState1 = {1, 1, 1, 1, 1, 1, 1};
    std::vector<int> nextState2 = {1, 1, 1, 1, 1, 1, 1};
    const int nextAction1 = op1->decide(nextState1, actions);
    const int nextAction2 = op2->decide(nextState2, actions);
    EXPECT_TRUE(nextAction1 == 0 || nextAction1 == 1);
    EXPECT_TRUE(nextAction2 == 0 || nextAction2 == 1);
}

TEST(IntersectionOperatorTest, QLearningImprovesActionChoice) {
    QLearningOperator op;
    const std::vector<int> state = {0, 1, 0, 1};
    const std::vector<int> actions = {0, 1};

    for (int i = 0; i < 1000; ++i) {
        op.learn(state, 0, -1.0, state, {});
        op.learn(state, 1, 1.0, state, {});
    }

    int preferredActionCount = 0;
    for (int i = 0; i < 200; ++i) {
        preferredActionCount += op.decide(state, actions) == 1;
    }
    EXPECT_GT(preferredActionCount, 170);
}

TEST(IntersectionOperatorTest, DQNImprovesActionChoice) {
    DeepRLOperator op;
    const std::vector<int> state = {0, 1, 0, 1};
    const std::vector<int> actions = {0, 1};

    // Initialize network, then train this state as a terminal contextual-bandit transition.
    op.decide(state, actions);
    for (int i = 0; i < 600; ++i) {
        op.learn(state, 0, -1.0, state, {});
        op.learn(state, 1, 0.0, state, {});
    }

    int preferredActionCount = 0;
    for (int i = 0; i < 200; ++i) {
        preferredActionCount += op.decide(state, actions) == 1;
    }
    EXPECT_GT(preferredActionCount, 185);
}

TEST(NeuralNetworkTest, BasicTraining) {
    std::vector<int> topology = {2, 4, 1};
    NeuralNetwork nn(topology, 0.1);
    
    const std::vector<double> input = {1.0, 0.5};
    const std::vector<double> target = {0.8};

    // Initial prediction
    auto p1 = nn.predict(input);
    EXPECT_EQ(p1.size(), 1u);
    
    // Train once
    nn.train(input, target);
    
    // Prediction should change
    auto p2 = nn.predict(input);
    EXPECT_NE(p1[0], p2[0]);
}
