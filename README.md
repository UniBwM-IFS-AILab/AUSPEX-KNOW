# AUSPEX-KNOW
World **Know**ledge Base

A knowledge base for all ***AUSPEX*** modules.

For detailed instructions see the main repository ***[AUSPEX](https://github.com/UniBwM-IFS-AILab/AUSPEX)***.
To cite ***AUSPEX-KNOW***, please use the following reference:
```
@article{10.3389/frobt.2025.1583479,
   AUTHOR={D{\"o}schl, Bj{\"o}rn  and Sommer, Kai  and Kiam, Jane Jean },
   TITLE={AUSPEX: An integrated open-source decision-making framework for UAVs in rescue missions},
   JOURNAL={Frontiers in Robotics and AI},
   VOLUME={Volume 12 - 2025},
   YEAR={2025},
   URL={https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1583479},
   DOI={10.3389/frobt.2025.1583479},
   ISSN={2296-9144}
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
