# UPPAAL Model and ROS 2 Executor Concurrency Analysis

This repository contains the artifacts related to our research on modeling and verifying ROS 2 callback concurrency using UPPAAL. It includes UPPAAL model templates, an automated model generation tool, and ROS 2 Python implementations of representative concurrency configurations.

## Repository Structure

```
|-- uppaal_templates/
|   |-- *.xml  # UPPAAL model templates for ROS 2 callback concurrency verification
|
|-- code_gen/
|   |-- uppaal_model_code_generation.py  # Automated UPPAAL model generation tool
|   |-- code_generation_files/  # Example files for model generation
|
|-- ros2_code/
|   |-- case1/  # Single-threaded executor with mutually exclusive callback group
|   |-- case2/  # Multi-threaded executor with mutually exclusive callback group
|   |-- case3/  # Multi-threaded executor with reentrant callback group
|   |-- external_node.py  # External node publishing topics
```

## Contents

### `uppaal_templates/`
This directory contains UPPAAL model templates that represent various ROS 2 callback concurrency configurations. The templates include:
- **Callback**
- **Callback Group**
- **Thread**
- **Executor**

These templates allow verification of callback execution properties in different scheduling scenarios.

### `code_gen/`
This directory provides an automated UPPAAL model generation tool:
- **`uppaal_model_code_generation.py`**: Generates UPPAAL models based on user-defined configurations.
- **`code_generation_files/`**: Contains example input files demonstrating the model generation process.

### `ros2_code/`
This directory includes ROS 2 Python implementations of three representative callback concurrency configurations:
- **Case 1**: Single-threaded executor with a mutually exclusive callback group.
- **Case 2**: Multi-threaded executor with a mutually exclusive callback group.
- **Case 3**: Multi-threaded executor with a reentrant callback group.

Additionally, an **external node** publishes topics to simulate interactions within a ROS 2 system.

## Getting Started

### Prerequisites
- **UPPAAL**: Download from [UPPAAL official site](https://www.uppaal.org/).
- **ROS 2**: Install according to the [ROS 2 documentation](https://docs.ros.org/en/).
- **Python 3**: Required for running `uppaal_model_code_generation.py`.

### Running the UPPAAL Model Generation Tool
```bash
cd code_gen
python3 uppaal_model_code_generation.py
```

### Running the ROS 2 Examples
1. Source your ROS 2 workspace:
   ```bash
   source /opt/ros/<distro>/setup.bash
   ```
2. Launch a ROS 2 example (e.g., Case 1):
   ```bash
   ros2 run ros2_code case1
   ```

## License
This repository is open-source and available under the [MIT License](LICENSE).

## Citation
If you use this repository in your research, please cite our paper:
> **[Your Paper Title]**  
> [Your Name], [Co-authors]  
> ACM SAC 2025, Italy

## Contact
For questions or contributions, please contact [your email].
