/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º ssd.c
Author: Zuo Lu 		Version: 1.0	Date:2017/04/06
Description:
Interface layer: to complete the IO request to obtain, and converted into the corresponding page-level SSD request

History:
<contributor>     <time>        <version>       <desc>                   <e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim       617376665@qq.com

*****************************************************************************************************************************/


int get_requests(struct ssd_info *);
__int64 find_nearest_event(struct ssd_info *);