/*****************************************************************************************************************************
This is a project on 3Dsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º fcl.h
Author: Zuo Lu 		Version: 2.0	Date:2017/02/07
Description:
fcl layer: remove other high-level commands, leaving only mutli plane;

History:
<contributor>		<time>			<version>       <desc>													<e-mail>
Zuo Lu				2017/04/06	      1.0		    Creat 3Dsim											lzuo@hust.edu.cn
Zuo Lu				2017/05/12		  1.1			Support advanced commands:mutli plane					lzuo@hust.edu.cn
Zuo Lu				2017/06/12		  1.2			Support advanced commands:half page read				lzuo@hust.edu.cn
Zuo Lu				2017/06/16		  1.3			Support advanced commands:one shot program				lzuo@hust.edu.cn
Zuo Lu				2017/06/22		  1.4			Support advanced commands:one shot read					lzuo@hust.edu.cn
Zuo Lu				2017/07/07		  1.5			Support advanced commands:erase suspend/resume			lzuo@hust.edu.cn
Zuo Lu				2017/07/24		  1.6			Support static allocation strategy						lzuo@hust.edu.cn
Zuo Lu				2017/07/27		  1.7			Support hybrid allocation strategy						lzuo@hust.edu.cn
Zuo Lu				2017/08/17		  1.8			Support dynamic stripe allocation strategy				lzuo@hust.edu.cn
Zuo Lu				2017/10/11		  1.9			Support dynamic OSPA allocation strategy				lzuo@hust.edu.cn
Jin Li				2018/02/02		  1.91			Add the allocation_method								li1109@hust.edu.cn
Ke wang/Ke Peng		2018/02/05		  1.92			Add the warmflash opsration								296574397@qq.com/2392548402@qq.com
Hao Lv				2018/02/06		  1.93			Solve gc operation bug 									511711381@qq.com
Zuo Lu				2018/02/07        2.0			The release version 									lzuo@hust.edu.cn
***********************************************************************************************************************************************/
struct ssd_info *dynamic_advanced_process(struct ssd_info *ssd, unsigned int channel, unsigned int chip);
//struct ssd_info *delete_from_ssd(struct ssd_info *ssd, unsigned int channel, struct sub_request * sub_req);
struct ssd_info *delete_from_channel(struct ssd_info *ssd, unsigned int channel, struct sub_request * sub_req);

struct ssd_info *make_same_level(struct ssd_info *, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);
struct ssd_info *compute_serve_time(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **subs, unsigned int subs_count, unsigned int command);

unsigned int find_mutliplane_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request ** sub_mutliplane_place, unsigned int command);
unsigned int find_read_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **subs, unsigned int state, unsigned int command);
unsigned int find_r_wait_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request ** sub_place, unsigned int command);
unsigned int find_static_write_sub_request(struct ssd_info *ssd, unsigned int channel, unsigned int chip, struct sub_request ** subs, unsigned int command);

Status check_req_in_suspend(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request * sub_plane_request);

Status find_level_page(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **sub, unsigned int subs_count);
Status go_one_step(struct ssd_info * ssd, struct sub_request **subs, unsigned int subs_count, unsigned int aim_state, unsigned int command);

Status services_2_r_read(struct ssd_info * ssd);
Status services_2_r_wait(struct ssd_info * ssd, unsigned int channel);
Status services_2_r_data_trans(struct ssd_info * ssd, unsigned int channel);
Status services_2_r_complete(struct ssd_info * ssd);
Status services_2_write(struct ssd_info * ssd, unsigned int channel);
Status service_advance_command(struct ssd_info *ssd, unsigned int channel, unsigned int chip, struct sub_request ** subs, unsigned int subs_count, unsigned int aim_subs_count, unsigned int command);