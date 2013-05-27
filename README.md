# Korus

Software for our mobile manipulation robot

## Installation

1. Install ros-groovy-desktop.
   
	```
	$ sudo apt-get install ros-groovy-desktop
	```
   
2. Get necessary dependencies

	```
	sudo apt-get install ros-groovy-ecl-devices ros-groovy-ecl-containers ros-groovy-ecl-formatters ros-groovy-ecl-command-line ros-groovy-ecl-streams ros-groovy-ecl-converters ros-groovy-ecl-geometry ros-groovy-ecl-sigslots ros-groovy-ecl-exceptions ros-groovy-ecl-threads ros-groovy-ecl-utilities ros-groovy-ecl-manipulators
	```
	
3. Get the rosinstaller

	```
	$ wget korus_groovy.rosinstall
	```
	
4. Set up your catkin workspace (we will be using the [yujin_tools](https://github.com/yujinrobot/yujin_tools/wiki/yujin-init) from here on)

	```
	$ yujin_init_workspace ./ korus_groovy.rosinstall
	```
	
5. Set-up the build directory

	```
	$ yujin_init_build
	```
	
6. Build everything

	```
	$ yujin_make
	```

## Running

- Visualising the model

	```
	$ roslaunch korus_meta view_model.launch
	```
	
- Launch the minimal setup for Korus (i.e. Goos, mechanism model, joint controller)

	```
	$ roslaunch korus_meta minimal.launch
	```
	
	or for simulation
	
	```
	$ roslaunch korus_meta minimal_sim.launch
	```
