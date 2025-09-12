# AUSPEX-KNOW
World **Know**ledge Base

A knowledge base for all ***AUSPEX*** modules.

For detailed instructions see the main repository ***[AUSPEX](https://github.com/UniBwM-IFS-AILab/AUSPEX)***.
To cite ***AUSPEX-KNOW***, please use the following reference:
```
@article{Doeschl-et-al:2025:AUSPEX,
  author = {Bj{\"o}rn D{\"o}schl and Kai Sommer and Jane Jean Kiam},
  title = {{AUSPEX: An Integrated Open-Source Decision-Making Framework for UAVs in Rescue Missions}},
  publisher = {TechRxiv.org},
  year = {2025},
  month = {March},
  doi = {10.36227/techrxiv.174123265.55724570/v1},
  url = {https://www.techrxiv.org/doi/full/10.36227/techrxiv.174123265.55724570/v1}
}
```

# Docker Container

Start the docker container ***[AUSPEX-VASA](https://github.com/UniBwM-IFS-AILab/AUSPEX-VASA)*** via:
```
runvasa
```
For attaching a second terminal to this container use:
```
vasabash
```

# Build

To build ***AUSPEX-KNOW*** you can use colcon build or the predefined alias:
```
build_know
```
> **_NOTE:_** This has to be done inside the ***AUSPEX-VASA*** docker container.

# Run

First run valkey server:
```
run_valkey
```
Then run ***AUSPEX-KNOW*** via:
```
run_know
```
> **_NOTE:_** This has to be done inside the ***AUSPEX-VASA*** docker container.

# Example data

For mocking UAV example data, you can run:
```
ros2 run auspex_knowledge drone_state_mock_publisher
```
> **_NOTE:_** This has to be done inside the ***AUSPEX-VASA*** docker container.
