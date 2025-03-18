<!--
SPDX-FileCopyrightText: Copyright (c) 2024 Infineon Technologies AG
SPDX-License-Identifier: MIT
-->

# IMR Software for IMD701A Motor Control 

<a href="https://www.infineon.com">
<img src="./assets/images/Logo.svg" align="right" alt="Infineon logo">
</a>
<br>
<br>

## Overview

<p>This is the official Infineon GitHub repository for ModusToolbox™ software used with the demo board for IMR IMD701A-based motor control.</p>
<p>The IMR motor control is responsible for receiving speed command, control the motor to actuate as required, and sending the speed measured by the position angle sensor as the encoder. Its main purpose is to drive a 3-phase BLDC motor with position angle sensor using Field Oriented Control (FOC) algorithm and PI controller based on fixed-point implementation optimized with hardware-accelerated CORDIC implementation.</p>

### Features

- Programmable motor controller integrating 5 V 32-bit microcontroller Arm® Cortex®-M0 XMC1404 with smart 3-phase gate driver IC 6EDL7141
- CAN bus communication with onboard CAN transceiver
- Onboard DIP switch for unique board identification
- Low Rds(on) in small package MOSFETs for the motor inverters
- Magnetic angle sensor as the position sensor / encoder on a <a href="https://www.infineon.com/cms/en/product/evaluation-boards/demo_imr_angle_sens_v1/">separate board</a>
- Absolute and incremental position possible with only Incremental Interface (IIF) i.e. no SSC / SPI interface is needed

### Reference hardware

<p>This software is meant to run on following reference hardware:</p>
- <a href="https://www.infineon.com/cms/en/product/evaluation-boards/demo_imr_mtrctrl_v1/">DEMO_IMR_MTRCTRL_V1 - Demo board for IMR motor control</a><br>
- <a href="https://www.infineon.com/cms/en/product/evaluation-boards/demo_imr_angle_sens_v1/">DEMO_IMR_ANGLE_SENS_V1 - Demo board for IMR encoder</a>

#### Featured Infineon Products 
<p>Following products are featured by the reference hardware:
<br>
<br>
<table style="width:100%">
  <tr>
    <th>Product</th>
    <th>Description</th>
  </tr>
  <tr>
    <td><a href="https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-control-ics/battery-supplied-bldc-motor-controller-ics/imd701a-q064x128-aa/">IMD701A-Q064X128-AA</a></td>
    <td>MOTIX™ fully programmable motor controller combining microcontroller and gate driver IC</td>
  </tr>
  <tr>
    <td><a href="https://www.infineon.com/cms/en/product/power/mosfet/n-channel/isz053n08nm6/">ISZ053N08NM6</a></td>
    <td>OptiMOS™ 6 N-channel power MOSFET 80 V 5.3 mOhm in PQFN 3.3 x 3.3</td>
  </tr>
  <tr>
    <td><a href="https://www.infineon.com/cms/en/product/transceivers/automotive-transceiver/automotive-can-transceivers/tle9351bvsj/">TLE9351BVSJ</a></td>
    <td>High speed CAN transceiver for CAN and CAN FD</td>
  </tr>
  <tr>
    <td><a href="https://www.infineon.com/cms/en/product/small-signal-transistors-diodes/diode/schottky-diodes/bas52-02v/">BAS52-02V</a></td>
    <td>45 V Silicon Schottky diode with low forward voltage at 200 mA</td>
  </tr>
  <tr>
    <td><a href="https://www.infineon.com/cms/en/product/sensor/magnetic-sensors/magnetic-position-sensors/angle-sensors/tle5012b-e1000/">TLE5012B E1000</a></td>
    <td>XENSIV™ 360° GMR magnetic angle sensor with SPI/IIF interface</td>
  </tr>
  <tr>
    <td><a href="https://www.infineon.com/cms/en/product/power/mosfet/p-channel/irlml6401/">IRLML6401</a></td>
    <td>12V Single P-Channel Power MOSFET in a SOT-23 package</td>
  </tr>
</table>
</p>
<br>

## Getting started

### How to import and use this repository
<ol>
<li> Install and start ModusToolbox™ and select a workspace to be used (tested with Version 3.3, and 3.4).
<li> Import the project with the import wizard by pressing 'File' – 'Import…'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_1.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Select 'ModusToolbox™' – 'Import Existing Application In-Place' and press 'Next'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_2.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Find the Project Location by pressing 'Browse…'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_3.png" >
    </picture>
    <br>
    &nbsp;
</li>
<li> Select the project folder accordingly and press 'Finish'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_4.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Wait until the project is fully imported. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_5.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Notice that additional folder 'mtb_shared' should be created (if there was none) when the import is completed. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_6.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Right click the project folder and select 'ModusToolbox™' followed by 'Library Manager 2...'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_7.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Press the 'Update' button <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_8.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> When the Update is completed the sucessful messages should be displayed. If the update failed, try it again by repressing the 'Update' button. If this also fails try to clean the project, before trying it again. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_9.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Before building the project it is recommended to clean it by pressing 'Clean Application'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_10.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Finally the project can be compiled by pressing 'Build Application'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_11.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Before flashing the project onto the board, connect the XMC™ Link Programming adapter using the 10-pin flat cable and <a href="./assets/DEMO_IMR_PROGADPTR_V1@e7eacb3013a-zip">the programming adapter</a> to provide power to the board. If the programming adapter is not available, additional wires need to be connected to the edge-card pins as indicated in the image below. <br><br>
	<picture>
        <img src="./assets/images/MTB_Import_12.png">
    </picture>
    <br>
	<picture>
        <img src="./assets/images/MTB_Import_13.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Finally to flash the project onto the board, use the green play button in ModusToolbox™ on the bottom left - 'Quick Panel' - 'Launches' - 'IMR_IMD701_MC Program (JLink)' to initiate the process.<br><br>
	<picture>
        <img src="./assets/images/MTB_Import_14.png">
    </picture>
    <br>
    &nbsp;
</li>
</ol>

## Additional information

Precise definition of the software and its features can be found in the close-to-code documentation on top of each file, at the specific function itself and in the software documentation.

### Related resources

- [Robotics development platform: Infineon Mobile Robot (IMR)](https://www.infineon.com/cms/de/applications/robotics/development-platform/)

### Licensing

Please see our [LICENSE](LICENSE) for copyright and license information.
