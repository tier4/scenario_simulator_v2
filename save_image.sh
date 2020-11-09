rm scenario_simulator.tar
docker build -t scenario_simulator .
docker save scenario_simulator -o scenario_simulator.tar