🚦🚗🚛🏍️ Traffic Simulator
==============================

**Road traffic simulation with ML-optimized traffic lights for maximum flow.**

![Traffic-Simulator](https://github.com/user-attachments/assets/d8e0898a-cf52-4609-9ef0-22802d56ef83)

## What is Traffic Simulator?

Traffic Simulator generates a random traffic network based on user-defined parameters, and uses machine-learning methods to optimize traffic lights for better flow.

Simply set your parameters and watch the magic happen ✨.

## 🚀Quick start
This project uses CMake for easy cross-platform builds.

### Clone/Download the repo
```cmd
git clone https://github.com/ninja7v/Traffic-Simulator.git
cd Traffic-Simulator
```
### Compile and run
```cmd
cmake -S . -B build
cmake --build build
```

Or even faster in Visual Studio Code: open the project and hit F5.

Once launched, a window pops up with a random traffic network running until you close it.

## 🧠Methods & results
Every intersection decides which incoming road gets the green light, once per traffic light period (2 s / boost). The control panel lets you pick the method:

- **Q-learning**: a table learns the value of each decision for each traffic situation, starting from random choices.
- **DQN (Deep Q-Network)**: a neural network replaces the table, so what is learned in one situation carries over to similar ones.
- **Heuristic**: hand-written rules. Keep the green while traffic flows, otherwise serve the busiest road whose vehicles can actually leave.

The learning methods see, for each incoming road, how many vehicles it holds, the speed of its first vehicle and how much it is used. Their reward is the number of vehicles that crossed the intersection minus a penalty for the vehicles waiting.

Flow in trips per simulated minute, over the first 10 minutes → after 50-60 minutes. Each seed is a different random network, the same for every method, with up to 70 vehicles:

| Method | Seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|
| DQN | 206 → **230** | 169 → **211** | 190 → **221** |
| Q-learning | 167 → **198** | 154 → **177** | 166 → **187** |
| Heuristic | 220 → 234 | 201 → 207 | 218 → 212 |
| Fixed cycle *(baseline)* | 160 → 168 | 160 → 169 | 212 → 211 |
| Random *(baseline)* | 116 → 129 | 106 → 136 | 143 → 147 |

Both learning methods improve the flow over time. DQN ends level with the hand-written heuristic. Q-learning beats the fixed cycle on 2 networks out of 3 but stays below the heuristic, because a table cannot generalize between similar situations.

To reproduce (`fixed`, `random`, `heuristic`, `qlearning` or `dqn`, then seed, minutes, window):
```cmd
./build/benchmark dqn 1 60 10
```

## 📖Documentation
This code is well commented for [Doxygen](https://www.doxygen.nl/index.html).
You can generate a complete documentation set using Doxygen.

More explanations and details are available in the project [Wiki](https://github.com/ninja7v/Traffic-Simulator/wiki)

## ✍️Contribute
Ideas, bug reports, or pull requests — all are welcome. Let’s build something awesome together!

Now it's your turn to generate traffic jams.

Thank you for using Traffic Simulator 🙂

> Just simulate !

## Licensing

This project is licensed under the MIT License - see the LICENSE.txt file for details.
