# example_code structure

## Teleo Folder
Contains the scripts used for teleoperating PSMs using Omni Device in AMBF simulator and dVRK.

## src folder

### 1. Agent
Contains the code for the GUI for the agent.  
*Note: The prompts are not contained in this folder.*

### 2. ECM_identify
Contains the code for using ECM to:
- Get the pose for the head 
- Get the pose for the root of the appendix  
*Uses a MRCNN (Mask R-CNN) model*

### 3. LfD (Learning from Demonstration)
Contains the code for:
- Registering the collected kinematic data
- Using the registered data to train a model to get desired manoeuvres
- Storing their primitives via DMP (Dynamic Movement Primitives)

### 4. PSMs
Contains the code for controlling PSMs in AMBF environment.

### 5. ambf_auto.py
Contains the code for system implementation.
