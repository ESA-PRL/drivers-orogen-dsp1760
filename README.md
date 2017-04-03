# DSP1760 optical gyroscope ROCK component

## Overview

This component uses the [DSP1760 optical gyroscope library](https://github.com/hdpr-rover/drivers-dsp1760) to output IMU and orientation messages to ROCK.

As there is no way to define the true initial orientation it is relative to the orientation in which the sensor was positioned when the component was started.

**Authors: Karl Kangur  
Contact: Martin Azkarate  
Affiliation: Automation and Robotics Laboratories, ESTEC, ESA**


## Installation

### Dependencies

This package depends on the following packages:

* [drivers/dsp1760](https://github.com/hdpr-rover/drivers-dsp1760)

### Building

In order to install, clone the latest version from this repository into your workspace under `drivers/orogen/dsp1760`, add the following line to `autoproj/manifest` under `layout:`

    - drivers/orogen/dsp1760

Execute the following to build the package:

    $ autoproj build


## Basic Usage

### dsp1760

#### Inputs

None

#### Outputs

* **`rotation`** (/base/samples/IMUSensors)

Gyro output (angular velocity), only rotation in Z axis is used.

* **`orientation_samples`** (/base/samples/RigidBodyState)

Timestamped IMU reading samples containing the "absolute" orientation (relative to initial orientation). Only rotation in Z axis is used.

* **`bias_samples`** (/base/samples/IMUSensors)

Gyro output (angular velocity) used to evaluate the bias

* **`bias_values`** (/double)

Output of bias (average of bias_samples)

#### Parameters

* **`port`** (/std/string)

The device port, by default: `/dev/ttyUSB0`.

* **`baudrate`** (/int)

The device baudrate, by default: `921600`.

* **`sampling_frequency`** (/double)

Nominal samping frequency of the sensor in Hz. Possible values are: `1`, `5`, `10`, `25`, `50`, `100`, `250`, `500`, `750`, `1000`, by default: `1000`.

* **`calibrate`** (/bool)

Start sensor calibration. Read during runtime so the calibration can be started at any time. By default set to `false`.

* **`calibration_samples`** (/int)

Number of samples to average over for bias calibration.

* **`bias`** (/double)

Bias of the gyroscope z-axis.
