/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º fcl.h
Author: Zuo Lu 		Version: 1.0	Date:2017/04/06
Description:
fcl layer: remove other high-level commands, leaving only mutli plane;

History:
<contributor>     <time>        <version>       <desc>                   <e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim       617376665@qq.com

*****************************************************************************************************************************/

struct sub_request * find_write_sub_request(struct ssd_info * ssd, unsigned int channel);
struct ssd_info *dynamic_advanced_process(struct ssd_info *ssd, unsigned int channel, unsigned int chip);
struct ssd_info *delete_from_channel(struct ssd_info *ssd, unsigned int channel, struct sub_request * sub_req);
struct sub_request * find_read_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die);
struct sub_request *find_interleave_twoplane_page(struct ssd_info *ssd, struct sub_request *one_page, unsigned int command);
struct ssd_info *make_same_level(struct ssd_info *, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);
struct ssd_info *compute_serve_time(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **subs, unsigned int subs_count, unsigned int command);

int services_2_r_cmd_trans_and_complete(struct ssd_info * ssd);
int make_level_page(struct ssd_info * ssd, struct sub_request * sub0, struct sub_request * sub1);
int find_level_page(struct ssd_info *, unsigned int, unsigned int, unsigned int, struct sub_request *, struct sub_request *);
int go_one_step(struct ssd_info * ssd, struct sub_request * sub1, struct sub_request *sub2, unsigned int aim_state, unsigned int command);
int services_2_r_wait(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag);
int services_2_r_data_trans(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag);
int services_2_write(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag);
int find_interleave_twoplane_sub_request(struct ssd_info * ssd, unsigned int channel, struct sub_request ** sub_request_one, struct sub_request ** sub_request_two, unsigned int command);




