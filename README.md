![image](https://github.com/tmacwg/Error-Identification-and-Compensation-of-a-Dual-robot-System/assets/33611353/29029b29-0767-4c99-a358-bbf8a96cdaea)


This is code for a paper that has been submitted to IEEE Transactions on Robotics, names as "Error Identification and Compensation of a Dual-robot Measuring and Machining System with an Integrated Visual Sensor, Gang Wang, Hua-yan Pu, Wen-long Li, Deng-yu Xiao, Cheng Jiang, Dong-fang Wang, Jun Luo, Han Ding"

Abstract: To address the growing demand for intelligent manufacturing in personalized, large-scale, and multi-variety production, multi-robot systems have demonstrated significant potential, as they can tackle complex tasks unattainable for a single robot. A critical challenge for enabling accurate cooperative operations is the precise calibration of relative poses and kinematic parameters of the robot system. In this study, building upon the preliminary calibration results of a dual-robot system, we introduce an error identification and compensation method hinged on data from an integrated visual sensor. Initially, the problem of multi-coordinate calibration for a dual-robot measuring and manufacturing system is formulated as a matrix equation, AXB=YCZ, where unknown matrices X, Y, and Z can be preliminary calibrated in advance. On this basis, a novel kinematic error transfer model for the dual-robot system is established to identify and compensate for errors in robot kinematic parameters and X, Y, Z, using the same input data as the preliminary calibration. As a result, the cooperative operation accuracy of the dual-robot system can be further enhanced. To demonstrate the feasibility and superiority of the proposed method, three calibration methods are compared to the proposed method through both simulations and experiments. The comparison results confirm the superiority of the proposed method in terms of accuracy and practicality.


The running environment is matlab 2021b or later.

1. Run "main_Dualrobot_3_english.m". 
2. The program will automatically complete simulation data generation, initial calibration, error compensation, error comparison and drawing of various calibration methods.
