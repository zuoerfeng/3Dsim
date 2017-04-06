/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º ssd.c
Author: Zuo Lu 		Version: 1.0	Date:2017/04/06
Description:
buff layer: only contains data cache (minimum processing size for the sector, that is, unit = 512B), mapping table (page-level);

History:
<contributor>     <time>        <version>       <desc>                   <e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim       617376665@qq.com

*****************************************************************************************************************************/

struct ssd_info *buffer_management(struct ssd_info *);
struct ssd_info *distribute(struct ssd_info *);
struct ssd_info *no_buffer_distribute(struct ssd_info *);
unsigned int transfer_size(struct ssd_info *, int, unsigned int, struct request *);
unsigned int size(unsigned int);
struct ssd_info *insert2buffer(struct ssd_info *, unsigned int, int, struct sub_request *, struct request *);
struct sub_request * creat_sub_request(struct ssd_info * ssd, unsigned int lpn, int size, unsigned int state, struct request * req, unsigned int operation);
int allocate_location(struct ssd_info * ssd, struct sub_request *sub_req);