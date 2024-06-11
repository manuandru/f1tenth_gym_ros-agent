# Commands to start Docker containers

From inside the folder where the `docker-compose.yml` file is located, run the following command to start the containers:

```bash
docker compose up --build
```

It will take some time to build all the code.

Do not close the terminal window, as it will stop the containers.

Open [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) to see the simulator.

Your algorithm should be in `car_node/car_node/main.py`.

To run them, open another terminal:

```bash
docker exec -it f1tenth_gym_ros-agent-sim-1 /bin/bash
ros2 run car_node car
```

## Client Server Approach

```bash
docker compose -f docker-compose-client-server.yml up --build
```

```bash
docker exec -it f1tenth_gym_ros-agent-sim-1 /bin/bash
ros2 run car_node car
```
