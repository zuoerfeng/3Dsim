/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º flash.h
Author: Zuo Lu 		Version: 1.8	Date:2017/08/17
Description:
flash layer: the original ssdsim this layer is not a specific description, it was their own package to achieve, not completed.

History:
<contributor>     <time>        <version>       <desc>													<e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim											617376665@qq.com
Zuo Lu			2017/05/12		  1.1			Support advanced commands:mutli plane					617376665@qq.com
Zuo Lu			2017/06/12		  1.2			Support advanced commands:half page read				617376665@qq.com
Zuo Lu			2017/06/16		  1.3			Support advanced commands:one shot program				617376665@qq.com
Zuo Lu			2017/06/22		  1.4			Support advanced commands:one shot read					617376665@qq.com
Zuo Lu			2017/07/07		  1.5			Support advanced commands:erase suspend/resume			617376665@qq.com
Zuo Lu			2017/07/24		  1.6			Support static allocation strategy						617376665@qq.com
Zuo Lu			2017/07/27		  1.7			Support hybrid allocation strategy						617376665@qq.com
Zuo Lu			2017/08/17		  1.8			Support dynamic stripe allocation strategy				617376665@qq.com
*****************************************************************************************************************************/

int erase_operation(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, unsigned int block);
int move_page(struct ssd_info * ssd, struct local *location, unsigned int move_plane, unsigned int * transfer_size);
int write_page(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, unsigned int active_block, unsigned int *ppn);

struct ssd_info *flash_page_state_modify(struct ssd_info *, struct sub_request *, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);