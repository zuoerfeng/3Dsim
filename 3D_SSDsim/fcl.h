/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º fcl.h
Author: Zuo Lu 		Version: 1.5	Date:2017/07/07
Description:
fcl layer: remove other high-level commands, leaving only mutli plane;

History:
<contributor>     <time>        <version>       <desc>											<e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim									617376665@qq.com
Zuo Lu			2017/05/12		  1.1			Support advanced commands:mutli plane			617376665@qq.com
Zuo Lu			2017/06/12		  1.2			Support advanced commands:half page read		617376665@qq.com
Zuo Lu			2017/06/16		  1.3			Support advanced commands:one shot program		617376665@qq.com
Zuo Lu			2017/06/22		  1.4			Support advanced commands:one shot read			617376665@qq.com
Zuo Lu			2017/07/07		  1.5			Support advanced commands:erase suspend/resume  617376665@qq.com
*****************************************************************************************************************************/
struct ssd_info *dynamic_advanced_process(struct ssd_info *ssd, unsigned int channel, unsigned int chip);
struct ssd_info *delete_from_channel(struct ssd_info *ssd, unsigned int channel, struct sub_request * sub_req);
struct ssd_info *make_same_level(struct ssd_info *, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);
struct ssd_info *compute_serve_time(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **subs, unsigned int subs_count, unsigned int command);

unsigned int find_mutliplane_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request ** sub_mutliplane_place, unsigned int command);
unsigned int find_read_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **subs, unsigned int state, unsigned int command);
unsigned int find_r_wait_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request ** sub_place, unsigned int command);
Status check_req_in_suspend(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request * sub_plane_request);

Status find_level_page(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **sub, unsigned int subs_count);
Status go_one_step(struct ssd_info * ssd, struct sub_request **subs, unsigned int subs_count, unsigned int aim_state, unsigned int command);

Status services_2_r_read(struct ssd_info * ssd);
Status services_2_r_wait(struct ssd_info * ssd, unsigned int channel);
Status services_2_r_data_trans(struct ssd_info * ssd, unsigned int channel);
Status services_2_r_complete(struct ssd_info * ssd);

int services_2_write(struct ssd_info * ssd, unsigned int channel);
Status service_advance_command(struct ssd_info *ssd, unsigned int channel, unsigned int chip, struct sub_request ** subs, unsigned int subs_count, unsigned int aim_subs_count, unsigned int command);