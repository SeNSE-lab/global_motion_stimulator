# Global Motion Stimulator

---

The Global Motion Stimulator is a tool for stimulating the whisker array of head fixed
rodents. The stimulator can traverse the array in any direction with a range of controllable speeds,
while an optical detector senses the duration of whisker contact. For more details see
Dorizan et al, **Journal of Neuroscience Methods 2022**.

---

## Hardware Assembly
- A model for the fully assembled device can be found [here](/3D_models/assembly)
- The BOM for all purchased parts can be found [here](/docs/BOM.xlsx)
- The designs for all 3D printed parts can be found [here](/3D_models/parts)
  - All parts were printed with PLA on an Ultimaker Pro 3
- The acrylic inserts were laser-cut from 1/4" clear acrylic. They have an outer
diameter of 3.25 mm. The tapered hole to hold the nitanol bar created was created using the
laser cutter on a lower setting
- A threaded 8mm aluminum rod is used to mount the device on a micro-manipulator with a M4 bolt
- Additional holes must be drilled in the encoder mounting in order to secure it to the stepper motor
It may be advisable to wrap in tape for better grip
- A circuit diagram can be found [here](/docs/schematic.pdf)
- The motor shield user manual can be found [here](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino)
  - Note the two Adafruit motor shields are stacked, the bottom configured to use the same
power source as the Arduino Mega, and the top to use its own source
- The fiber optic user manual can be found [here](https://www.bannerengineering.com/us/en/products/part.87591.html). A 10 kOhm load
was used

---

## Software

### Arduino

#### Installation
- This project uses a modified 
[Adafruit Motor Shield Library](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/install-software) 
which can be found in the [arduino/libraries](/arduino/libraries) folder in this project. It is reccomended that you install 
the library using the instructions on the [Adafruit website](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/install-software) then 
replace the .h and .cpp files with the ones contained in this project. This allows you to automatically install all 
dependancies (select "Install All" when using the library manager to install automatically)
  - The library typically installs either in Program Files (x86)/Arduino/libraries or Documents/Arduino/libraries
- Ensure you alter the paths at the top of the `main_slider_control.ino` script to reflect the correct
local locations for the `data_struct_lib.ino` and `timer_lib.ino` files

### Python

#### Required libraries
- pyserial
- pandas
- matplotlib

#### Instructions
`sliderlib.py` contains the main functionality for controlling the global motion stimulator device. 
Documentation can be found in comments or [here](/docs/sliderlib_documentation.pdf) 
Two example jupyter notebooks are provided with instructions in the notebooks for controlling the device. 
The `calibration_notebook` is used to assist in calibrating speeds for each angle, though calibrations should 
be verified with slider data recorded from the data acquisition system. The experiment notebook is to be used 
when running experiments.




