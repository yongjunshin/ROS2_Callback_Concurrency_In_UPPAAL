# ROS2_Callback_Concurrency_In_UPPAAL

This repository contains the artifacts related to our research titled "Verifying ROS 2 Callback Concurrency Using UPPAAL Model Checker" (under review). 

It includes UPPAAL model templates, an automated model generation tool, and ROS 2 Python implementations of representative cases of callback concurrency configurations.


## Contact
⚠️ **This repository is not hosted by the authors because the paper associated with it is under double-blinded review.** ⚠️ 

The repository will be moved to a permanent location after the review process. 


## Repository Structure

```
|-- uppaal_templates/
|   |-- ros2_concur_v1.0_template.xml  # UPPAAL model templates for ROS 2 callback concurrency verification
|
|-- code_gen/
|   |-- uppaal_model_code_generation.py  # Automated UPPAAL model generation tool
|   |-- code_generation_files/  # Example files for model generation
|
|-- ros2_code/src/
|   |-- single_thread/  # Case 1
|   |-- multi_thread/   # Case 2 and 3
```

## Contents

### `uppaal_templates/`
This directory contains UPPAAL model templates that represent various ROS 2 callback concurrency configurations. The templates include:
- **Callback**
- **Callback Group**
- **Thread**
- **Executor**

These templates allow verification of callback execution properties in different scenarios.

### `code_gen/`
This directory provides an automated UPPAAL model generation tool:
- **`uppaal_model_code_generation.py`**: Generates UPPAAL models based on user-defined node specification in YAML.
- **`code_generation_files/`**: Contains example input files demonstrating the model generation process.

### `ros2_code/`
This directory includes ROS 2 Python implementations of three representative callback concurrency configurations:
- **Case 1**: Single-threaded executor with a mutually exclusive callback group.
- **Case 2**: Multi-threaded executor with a mutually exclusive callback group.
- **Case 3**: Multi-threaded executor with a reentrant callback group.

Additionally, an **external node** publishes topics to simulate interactions within a ROS 2 system.

