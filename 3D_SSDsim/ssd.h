/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º ssd.h
Author: Zuo Lu 		Version: 1.2	Date:2017/06/12
Description: System main function c file, Contains the basic flow of simulation.
Mainly includes: initialization, make_aged, pre_process_page three parts

History:
<contributor>     <time>        <version>       <desc>									<e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim							617376665@qq.com
Zuo Lu			2017/05/12		  1.1			Support advanced commands:mutli plane   617376665@qq.com
Zuo Lu			2017/06/12		  1.2			Support advanced commands:half page read   617376665@qq.com
*****************************************************************************************************************************/
//#define DEBUG

void main();
void trace_output(struct ssd_info* );
void statistic_output(struct ssd_info *);
void free_all_node(struct ssd_info *);

struct ssd_info *make_aged(struct ssd_info *);
struct ssd_info *pre_process_write(struct ssd_info *ssd);
struct ssd_info *process(struct ssd_info *);
struct ssd_info *simulate(struct ssd_info *);

void file_assert(int error, char *s);
void alloc_assert(void *p, char *s);
void trace_assert(_int64 time_t, int device, unsigned int lsn, int size, int ope);
