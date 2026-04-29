# AUSPEX-KNOW
A **Know**ledge base for multi-UAV decision-making frameworks.

For detailed installation and setup instructions of the complete framework, see the main repository **[AUSPEX](https://github.com/UniBwM-IFS-AILab/AUSPEX)**.

To cite **AUSPEX-KNOW**, please use the following reference:
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

Start the docker container **[AUSPEX-VASA](https://github.com/UniBwM-IFS-AILab/AUSPEX-VASA)** via:
```
vasa_shell
```
For attaching a second terminal to this container use:
```
vasa_shell
```

# Build

To build **AUSPEX-KNOW** you can use colcon build or the predefined alias:
```
know_build
```
> **NOTE:** This has to be done inside the **AUSPEX-VASA** docker container.

# Run

First run valkey server:
```
valkey_run
```
Then run **AUSPEX-KNOW** via:
```
know_run
```
> **NOTE:** This has to be done inside the **AUSPEX-VASA** docker container.

# Example data

For example data, you can use the mock publishers in the *AUSPEX-KNOW\src\auspex_knowledge\auspex_knowledge\mock* folder.

> **NOTE:** This has to be done inside the **AUSPEX-VASA** docker container.
