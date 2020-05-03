# Lego Robot

## Brain Setup

Need to figure what that brain is. Probably subscribes and publishes to a bunch of queues to give and receive information.

## Robot Driver (./motion-driver)

This module is meant to run the robot's mechanical components.

- [ ] Validate move X will turn some angle when not using PID... which is strange.... pid controler related perhaps.
- [ ] Finish working on the IK testing
- [ ] Make the robot arm position a "set point" and every X time, resynch to the set point... so MQTT would just "set the point". (maybe lower wiggle?)

### Hardware (playing with Legos :))

- [X] ~~Create a rotary gripper (Adding a 6th degree of freedom to the gripper - currently has 5 dof) with the turntable~~
- [X] ~~Calibrate both IMU sensors~~

### Software

- [ ] Motion Feedback (via mqtt pubsub reversed so the client knows the bot is in place)
- [ ] Error Handling to MQTT error queue (in custom error handlers)
- [ ] Go to home position (avoiding obstacles)
- [ ] Follow Tracked Object?...
- [ ] Build architecture with IOT Hub for deployment/management and collection of events, BrainWave for inferencing
  - [ ] Events are : position, movement orders, inferencing output
- [ ] IOT Edge Connection and streaming of events :
  - [ ] Error Handling
  - [ ] Motion commands
  - [ ] Pick commands
  - [ ] Objects seen
  - [ ] Wrongfully picked object - will go to a DNN training feedback loop (create new training examples)

## Decision Center

- [ ] Find Object
- [ ] Track Object
  - [ ] Detect Object
  - [ ] Track it
- [ ] Follow Tracked Object
  - [ ] Keep tracked object in field of view
    - [ ] Loop
      - [ ] If drifting to the left, turn + phi by a few deg, if drift to right, thrn - phi).
      - [ ] Advance towards object /bot/base/move/forward for 1-2 seconds, then reloop until distance to object is within ~10-20cm
    - [ ] fixed distance if static configuration, using /bot/base/move/XYphi)
- [ ] Pick Object with end effector /bot/pickObject
  - [ ] Move base to standoff
  - [ ] Run inverse kinatics to find the exact position... to run next step
  - [ ] move to object
  - [ ] pick object (close gripper)
  - [ ] move back above in standoff
  - [ ] await further instructions
  - [ ] Drop object (open gripper /bot/gripper/open)
  - [ ] Go back to home position
  - [ ] For each pair of main steps :
    - [ ] Generate list of detailed robot configuration (number of position per step TBD, will do trial and error)
    - [ ] For each robot config step, move joints and wheels to position (or use velocities) to next configuration (end effector + mobile base)

## Cognitive Dialogue System

### Text to Speech (Neural Speech with Voice Print)

### Speech to Text

#### Text-to-speech

- [ ] Need to create a container to run demo python code to validate it can receive code and utilize Azure to do TTS. See containers offered on prem to see if one works on a PI or Jetson.

#### Speech-to-text

- [ ] Need to test Azure STT.

### Intent Understanding (LUIS)

- [ ] Need to test LUIS and LUIS in a container.

### Bot Framework Module

- [ ] TBD - add personnality module

## Visual Perception (./object-detection)

### Environment Understanding (Mapping with visual information)
