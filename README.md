# Installation
## Prerequisites
### GTSAM

GTSAM can be installed from source. Installation from apt is outdated and does not provide the CppUnitLite lib required for the package's tests.

Install the build dependencies:
```bash
sudo apt update
sudo apt install build-essential cmake git libboost-all-dev libeigen3-dev libtbb-dev
```

Clone GTSAM outside the ROS workspace so that `colcon` does not discover it as another workspace package:

```bash
mkdir -p ~/src
git clone --branch release/4.3a.2 --depth 1 https://github.com/borglab/gtsam.git ~/src/gtsam-4.3a.2
cd ~/src/gtsam-4.3a.2
```

Configure and build:

```bash
cd ~/src/gtsam

cmake -S . -B build \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_INSTALL_PREFIX=/usr/local
# Build using all available CPU cores
cmake --build build -j "$(nproc)"

# Optional: run the tests
cmake --build build --target check -j "$(nproc)"
```

Then install and refresh the shared-library cache:

```bash
sudo cmake --install build
sudo ldconfig
```

### Geographiclib
Geographiclib can be installed from [source](https://github.com/geographiclib/geographiclib) if you like the hassle, or easily from apt.
```bash
sudo apt update
sudo apt install libgeographiclib-dev
```
If apt saids it could not locate the package, enable universe:
```bash
sudo add-apt-repository universe
sudo apt update
sudo apt install libgeographiclib-dev
```

### Building the package
First, update the submodule by going to `smarc2` and run the bash script to get the submodules updated.
```bash
cd <path_to_your_workspace>/src/smarc2
./scripts/get-submodules.sh
```
Then colcon build the package:
```bash
cd <path_to_your_workspace>
colcon build --packages-select hydrobatic_localization
source install/setup.bash
```

---
# Hydrobatic Localizaiton
This project is a localization scheme implemented with the GTSAM framework for the AUV SAM, the project contains custom factors for the Doppler Velocity Logger, Barometer and the Motion Model, acounting for the offset of the sensors with respect to the base link. In order to build the project the following needs to be installed:
* GTSAM
* Geographiclib for the lat/lon to UTM converions: https://geographiclib.sourceforge.io/C++/doc/install.html
* Follow the instrucitons and build the smarc_modelling submodule in order to run the motion model.

In oder to run the localizer simply run the launch file, this will use the default parameters.
```
ros2 launch hydrobatic_localization state_estimator.launch 
```
The launch file has 2 variables that can be specified, namely a boolean **use_motion_model**, which specifies if the ros node should subscribe to the contorl inputs and the gtsam graph should add the motion model factor to the graph. The deafault is set to **true**. The other parameter is **inference_strategy**, which specifies if the gtsam graph should use *fullsmoothing*, *ISAM2* or *fixedlagsmoothining*, the flags are **FullSmoothing**, **ISAM2** and **FixedLagSmoothing** respectivly, with fullsmoothing being the default. The **kf_interval_hz** parameters sets the rate at which keyframes optimizations are done, and **use_sensor_covariance** specifies if the covariances from the GPS and DVL drivers should be used instead of the ones in the config file. In order to specify a config file either use the name of the file 

A example for running with the motion model turned off and with the fixedlagsmoother is:
```
ros2 launch hydrobatic_localization state_estimator.launch use_motion_model:=false inference_strategy:=FixedLagSmoother kf_interval_hz:=10 use_sensor_covariance:=true

```
