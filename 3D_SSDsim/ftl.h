/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º ftl.h
Author: Zuo Lu 		Version: 1.3	Date:2017/06/16
Description:
ftl layer: can not interrupt the global gc operation, gc operation to migrate valid pages using ordinary read and write operations, remove support copyback operation;

History:
<contributor>     <time>        <version>       <desc>									<e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim							617376665@qq.com
Zuo Lu			2017/05/12		  1.1			Support advanced commands:mutli plane   617376665@qq.com
Zuo Lu			2017/06/12		  1.2			Support advanced commands:half page read   617376665@qq.com
Zuo Lu			2017/06/16		  1.3			Support advanced commands:one shot program   617376665@qq.com
*****************************************************************************************************************************/

struct ssd_info *pre_process_page(struct ssd_info *ssd);
struct local *find_location(struct ssd_info *ssd, unsigned int ppn);
struct ssd_info *get_ppn(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, struct sub_request *sub);

unsigned int gc(struct ssd_info *ssd, unsigned int channel, unsigned int flag);
unsigned int get_ppn_for_pre_process(struct ssd_info *ssd, unsigned int lsn);
unsigned int get_ppn_for_gc(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane);
unsigned int find_ppn(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, unsigned int block, unsigned int page);

int gc_for_channel(struct ssd_info *ssd, unsigned int channel);
int delete_gc_node(struct ssd_info *ssd, unsigned int channel, struct gc_operation *gc_node);
Status get_ppn_for_normal_command(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request * sub);
int  find_active_block(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane);

int gc_direct_erase(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die);
int uninterrupt_gc(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die);
int get_ppn_for_advanced_commands(struct ssd_info *ssd, unsigned int channel, unsigned int chip, struct sub_request * * subs, unsigned int subs_count, unsigned int command);










