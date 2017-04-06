SSDsim is a simulator which can simulate the behaviors of hardware and software of SSD. It provides the simulated results of performance, endurance and energy consumption under each kind of workloads.
SSDsim is created by Yang Hu in 2009, 2010 and improved under the help of Zhiming Zhu¡¢Shuangwu Zhang¡¢Chao Ren¡¢Hao Luo in 2011. We will continue to add new modules and functions for SSDsim. If you have any question, advice or requirement, please let us know. You can email to yanghu@foxmail.com.

Step:
1.	Use visual studio 2008 to compile the code, and create a executable file.
2.	Execute the executable file and input parameter file, trace file, output file and statistic file.

Note: 
1.	The format of the trace file must be ASCII: 
¡°arrival time of the request¡± ¡°logical device number¡± ¡°lsn¡± ¡°size¡± ¡°operation code¡±
2.	The unit of arrival time of request is nanosecond. And the unit of size is sector.
3.	The bottom line of the trace file must not be linefeed.
