# Modeling-Development-and-Design-of-an-Inverted-Pendulum-System-
Repository for a bachelor's project by a group of students from the Norwegian University of Science and Technology

## About The Project
The inverted pendulum has made its mark as a staple in the field of control theory, making an appearance in classic mechanical systems such as the Furuta pendulum and the cart-pendulum system. This thesis focuses on the reaction wheel pendulum, more specifically the modeling, development and design of an inverted spherical pendulum actuated by two perpendicular reaction wheels. The system is developed with the overarching objective of functioning as an interactive learning platform for testing and experimenting with various control configurations.

The modeling of the system is centered around the derivation of an expression for the dynamics, which is done through the utilization of the Euler-Lagrange equations. The design process integrates a hardware and mechanical design approach, aiming to achieve optimum functionality while minimizing expenditure. Fabrication of the mechanical components entails 3D printing, and the hardware components are intricately connected onto a bespoke PCB. The implementation of the operational functionalities, e.g., Euler angle estimation and control system algorithms, is executed through the usage of the ESP32 family-based microcontroller. Additionally, wireless communication between the microcontroller and a Python-based user interface is established, making the product intuitive and user-friendly. 



<div style="display: flex; justify-content: center;">
  <img src="https://github.com/Ludvigvart/Modeling-Development-and-Design-of-an-Inverted-Pendulum-System-/assets/97682577/c611afdd-bf9d-439f-9153-e396584b6c3e" alt="dual_axis_system" width="300">
</div>

## Programs Used
Fusion 360 by Autodesk was used to design all 3D-printed components for the system. The microcontroller program was written in Arduino C. Altium Designer was used to design the custom PCB.
Python 3.11 was used to create the PC-program. 
* [<a href="https://www.autodesk.no/products/fusion-360/overview"><img src="https://i0.wp.com/www.autodesk.com/products/fusion-360/blog/wp-content/uploads/2019/06/fusion-logo.png?ssl=1"  width="150" height="40"></a>](https://www.autodesk.no/products/fusion-360/overview)
* [<a href="https://www.arduino.cc/en/software"><img src="https://images.squarespace-cdn.com/content/v1/5f4fc1d00dea6b17b03f63ad/1613610070709-1IF1A6I1W246K9U3NB1P/Screen+Shot+2021-02-17+at+18.00.37.png"  width="150" height="40"></a>](https://www.arduino.cc/en/software)
* [<a href="https://www.altium.com/altium-designer"><img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcSSSeZkUI6PymGaxIRjBVsYQfEBIH03PZB0Vg&usqp=CAU"  width="150" height="40"></a>](https://www.altium.com/altium-designer)
* [<a href="https://www.python.org/"><img src="https://www.python.org/static/community_logos/python-logo-master-v3-TM-flattened.png"  width="150" height="40"></a>](https://www.python.org/)



## Notes Regarding Usage
When uploading new code to microcontroller with a USB cable, it is important that the power switch on the system is in the OFF position. With the power switch ON and a USB cable connected, it is likely that the microcontroller will be damaged, as 5 V then will be supplied from two different sources. 
When charging the batteries, it is recommended that a voltage between 12.6 - 13 V is supplied with a current of approximately 2 A. We suggest that the batteries should always have a charge between 11.1 – 12.6 V to retain suitable motor performance. 

## Notes Regarding Included Files
The finalized microcontroller code can be accessed in the designated Arduino code folder. The CAD models and adjoining G-code files of the system's 3D-printed parts are made accessible within the CAD folder. The design files for the custom PCB are available under the PCB folder. A compiled executable "Inverted Pendulum Interface.exe" of the PC-program is also provided, along the full Python code. Regarding the PC-program it is worth mentioning that it’s intended to work on any Windows 10/11 PC with a screen resolution of 1920x1080 and a scaling factor set to 100%.  







