/*****************************************************************************************************************************
This is a project on 3Dsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName： ftl.c
Author: Zuo Lu 		Version: 2.0	Date:2017/02/07
Description: 
ftl layer: can not interrupt the global gc operation, gc operation to migrate valid pages using ordinary read and write operations, remove support copyback operation;

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

#define _CRTDBG_MAP_ALLOC

#include <stdlib.h>
#include <crtdbg.h>

#include "initialize.h"
#include "ssd.h"
#include "flash.h"
#include "buffer.h"
#include "interface.h"
#include "ftl.h"
#include "fcl.h"

extern int secno_num_per_page, secno_num_sub_page;
/******************************************************************************************下面是ftl层map操作******************************************************************************************/

/*********************************************************************
*pre_process_page() handle all read request in advance and established 
*lpn<-->ppn of read request in advance ,in order to pre-processing trace 
*to prevent the read request is not read the data;
**********************************************************************/
struct ssd_info *pre_process_page(struct ssd_info *ssd)
{
	int fl = 0;
	unsigned int device, lsn, size, ope;
	unsigned int largest_lsn, ppn;
	unsigned int lpn, full_page, last_lpn, first_lpn, mask, state;
	unsigned int offset1 = 0, offset2 = 0;
	unsigned int i = 0, j, k, p;
	char buffer_request[200];
	struct local *location;
	__int64 time;
	errno_t err;
	unsigned int page_num;

	printf("\n");
	printf("begin pre_process_page.................\n");

	if ((err = fopen_s(&(ssd->tracefile), ssd->tracefilename, "r")) != 0)  
	{
		printf("the trace file can't open\n");
		return NULL;
	}

	full_page = ~(0xffffffff << (ssd->parameter->subpage_page));
	/*Calculate the maximum logical sector number for this ssd*/
	largest_lsn = (unsigned int)((ssd->parameter->chip_num*ssd->parameter->die_chip*ssd->parameter->plane_die*ssd->parameter->block_plane*ssd->parameter->page_block*secno_num_per_page)*(1 - ssd->parameter->overprovide));
	page_num = ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->plane_die*ssd->parameter->die_chip*ssd->parameter->chip_num;

	while (fgets(buffer_request, 200, ssd->tracefile))
	{
		sscanf_s(buffer_request, "%I64u %d %d %d %d", &time, &device, &lsn, &size, &ope);
		fl++;
		trace_assert(time, device, lsn, size, ope);                       

		//防止trace访问超过设置最大的扇区号
		lsn = lsn%largest_lsn;

		//预处理的时候建立负载感知的表
		if (ssd->parameter->allocation_scheme == HYBRID_ALLOCATION)
		{	
			//进行4kb对齐
			size = ((lsn + size - 1) / secno_num_sub_page - (lsn) / secno_num_sub_page + 1) * secno_num_sub_page;
			lsn /= secno_num_sub_page;
			lsn *= secno_num_sub_page;

			lpn = lsn / secno_num_per_page;
			last_lpn = (lsn + size - 1) / secno_num_per_page;
			first_lpn = lsn / secno_num_per_page;   //计算lpn
			
			while (lpn <= last_lpn)
			{
				if (lpn > page_num)
				{
					printf("error\n");
					getchar();
				}
				if (ope == READ)
					ssd->dram->map->map_entry[lpn].read_count++;

				else if (ope == WRITE)
					ssd->dram->map->map_entry[lpn].write_count++;

				lpn++;
			}
		}
		
		//进行预处理，即处理所有的读请求，将读请求转换为写请求，同时在映射表中记录
		if (ope == 1)
		{
			//进行4kb对齐
			size = ((lsn + size - 1) / secno_num_sub_page - (lsn) / secno_num_sub_page + 1) * secno_num_sub_page;
			lsn /= secno_num_sub_page;
			lsn *= secno_num_sub_page;

			lpn = lsn / secno_num_per_page;
			last_lpn = (lsn + size - 1) / secno_num_per_page;
			first_lpn = lsn / secno_num_per_page;   //计算lpn

			while (lpn <= last_lpn)
			{
				mask = ~(0xffffffff << (ssd->parameter->subpage_page));   //掩码表示的是子页的掩码
				state = mask;

				if (lpn == first_lpn)
				{
					//offset表示state中0的个数，也就是第一个页中缺失的部分
					offset1 = ssd->parameter->subpage_page - (((lpn + 1)*secno_num_per_page - lsn) / secno_num_sub_page);
					state = state&(0xffffffff << offset1);
				}
				if (lpn == last_lpn)
				{
					offset2 = ssd->parameter->subpage_page - ((lpn + 1)*secno_num_per_page - (lsn + size)) / secno_num_sub_page;
					state = state&(~(0xffffffff << offset2));
				}

				if (state > 15)
					printf("error\n");

				if (lpn > ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->plane_die*ssd->parameter->die_chip*ssd->parameter->chip_num)
					printf("error\n");

				//state表示请求的状态位
				if (ssd->dram->map->map_entry[lpn].state == 0)
				{
					//ppn = get_ppn_for_pre_process(ssd, lsn);
					ppn = get_ppn_for_pre_process(ssd, lpn);
					location = find_location(ssd, ppn);
					ssd->pre_all_write++;
					ssd->dram->map->map_entry[lpn].pn = ppn;
					ssd->dram->map->map_entry[lpn].state = state;   
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].pre_write_count++;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].lpn = lpn;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].valid_state = ssd->dram->map->map_entry[lpn].state;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].free_state = ((~ssd->dram->map->map_entry[lpn].state)&full_page);
				
					free(location);
					location = NULL;
				}
				else if (ssd->dram->map->map_entry[lpn].state>0)
				{
					ppn = ssd->dram->map->map_entry[lpn].pn;
					location = find_location(ssd, ppn);
					ssd->dram->map->map_entry[lpn].state |= state;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].valid_state = ssd->dram->map->map_entry[lpn].state;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].free_state = ((~ssd->dram->map->map_entry[lpn].state)&full_page);

					free(location);
					location = NULL;
				}
				lpn++;
			}
		}
	}

	if (ssd->parameter->allocation_scheme == HYBRID_ALLOCATION)
	{
		//当读写统计的次数完成了之后，开始计算多读还是多写
		for (i = 0; i < page_num; i++)
		{

			if ((ssd->dram->map->map_entry[i].read_count != 0) || (ssd->dram->map->map_entry[i].write_count != 0))
			{
				if ((ssd->dram->map->map_entry[i].read_count / (ssd->dram->map->map_entry[i].read_count + ssd->dram->map->map_entry[i].write_count)) >= 0.8)				//读的次数超过总次数的80%
					ssd->dram->map->map_entry[i].type = READ_MORE;
				else
					ssd->dram->map->map_entry[i].type = WRITE_MORE;
			}
		}
	}
	
	printf("\n");
	printf("pre_process is complete!\n");

	fclose(ssd->tracefile);

	for (i = 0; i < ssd->parameter->channel_number; i++)
		for (p = 0; p < ssd->parameter->chip_channel[i]; p++)
			for (j = 0; j<ssd->parameter->die_chip; j++)
				for (k = 0; k<ssd->parameter->plane_die; k++)
				{
		fprintf(ssd->outputfile, "chip:%d,die:%d,plane:%d have free page: %d\n", p, j, k, ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].free_page);
		fflush(ssd->outputfile);
				}

	return ssd;
}


struct allocation_info* pre_process_allocation(struct ssd_info *ssd, unsigned int lpn)
{	
	
	unsigned int channel = 0, chip = 0, die = 0, plane = 0;
	unsigned int channel_num = 0, chip_num = 0, die_num = 0, plane_num = 0;
	
	struct allocation_info * allocated_info = (struct allocation_info *)malloc(sizeof(struct allocation_info));
	alloc_assert(allocated_info, "allocated_info");
	memset(allocated_info, 0, sizeof(struct allocation_info));

	channel_num = ssd->parameter->channel_number;
	chip_num = ssd->parameter->chip_channel[0];
	die_num = ssd->parameter->die_chip;
	plane_num = ssd->parameter->plane_die;

	if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION || ssd->parameter->allocation_scheme == HYBRID_ALLOCATION)                           /*Dynamic way to get ppn*/
	{
		if (ssd->parameter->dynamic_allocation == CHANNEL_DYNAMIC_ALLOCATION)						  //assign priority：channel>die>plane
		{

			channel = ssd->token;
			ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;
			chip = ssd->channel_head[channel].token;
			ssd->channel_head[channel].token = (chip + 1) % ssd->parameter->chip_channel[0];
			die = ssd->channel_head[channel].chip_head[chip].token;
			ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
			plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;

			ssd->page_count++;

			if (ssd->parameter->flash_mode == TLC_MODE)
			{
				if (ssd->page_count % PAGE_INDEX == 0)                                      //tlc模式下，首先写同个die的三个页，写满了再去写另外的plane
				{
					ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
					ssd->page_count = 0;
				}
			}
			else if (ssd->parameter->flash_mode == SLC_MODE)
			{
				ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
				ssd->page_count = 0;
			}
		}
		else if (ssd->parameter->dynamic_allocation == PLANE_DYNAMIC_ALLOCATION)																		//assign priority：plane>channel>die
		{
			channel = ssd->token;
			chip = ssd->channel_head[channel].token;
			die = ssd->channel_head[channel].chip_head[chip].token;
			plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
			ssd->page_count++;

			if (ssd->parameter->flash_mode == TLC_MODE)
			{
				if (ssd->page_count % PAGE_INDEX == 0)
				{
					ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
					if (plane == (ssd->parameter->plane_die - 1))
					{
						ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;
						ssd->channel_head[channel].token = (ssd->channel_head[channel].token + 1) % ssd->parameter->chip_channel[0];
						if (ssd->token == 0)
							ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = (die + 1) % ssd->parameter->die_chip;
						else
							ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = die;
					}
					ssd->page_count = 0;
				}
			}
			else if (ssd->parameter->flash_mode == SLC_MODE)
			{
				ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
				if (plane == (ssd->parameter->plane_die - 1))
				{
					ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;
					ssd->channel_head[channel].token = (ssd->channel_head[channel].token + 1) % ssd->parameter->chip_channel[0];
					if (ssd->token == 0)
						ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = (die + 1) % ssd->parameter->die_chip;
					else
						ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = die;
				}
				ssd->page_count = 0;
			}
		}
		else if (ssd->parameter->dynamic_allocation == STRIPE_DYNAMIC_ALLOCATION || ssd->parameter->dynamic_allocation == OSPA_DYNAMIC_ALLOCATION || ssd->parameter->dynamic_allocation == POLL_DISTRANCE_ALLOCATION)
		{
			//本次操作对应的location
			channel = ssd->token;
			chip = ssd->channel_head[channel].token;
			die = ssd->channel_head[channel].chip_head[chip].token;
			plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;

			//更新下次操作的令牌
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
			if (ssd->channel_head[channel].chip_head[chip].die_head[die].token == 0)
			{
				ssd->channel_head[channel].token = (ssd->channel_head[channel].token + 1) % ssd->parameter->chip_channel[0];
				ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = (die + 1) % ssd->parameter->die_chip;
				if (ssd->channel_head[channel].token == 0)
					ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;
			}
		}
	}
	else if (ssd->parameter->allocation_scheme == STATIC_ALLOCATION)
	{
		//预处理的分配，是直接去写，所以应该分配到对应的plane上,按照静态分配方式去进行(分tlc/slc mode)
		if (ssd->parameter->flash_mode == TLC_MODE)
		{
			switch (ssd->parameter->static_allocation)
			{
			case PLANE_STATIC_ALLOCATION:													 //1.plane>superpage>channel>chip>die 
			{
				plane = lpn % plane_num;
				channel = (lpn / (plane_num*PAGE_INDEX)) % channel_num;
				chip = (lpn / (plane_num*PAGE_INDEX*channel_num)) % chip_num;
				die = (lpn / (plane_num*PAGE_INDEX*channel_num*chip_num)) % die_num;
				break;
			}
			case SUPERPAGE_STATIC_ALLOCATION:												//2.superpage>plane>channel>chip>die
			{
				plane = (lpn / PAGE_INDEX) % plane_num;
				channel = (lpn / (plane_num*PAGE_INDEX)) % channel_num;
				chip = (lpn / (plane_num*PAGE_INDEX*channel_num)) % chip_num;
				die = (lpn / (plane_num*channel_num*chip_num*PAGE_INDEX)) % die_num;
				break;
			}
			case CHANNEL_PLANE_STATIC_ALLOCATION:											//3.channel>chip>plane>superpage>die
			{
				channel = lpn % channel_num;
				chip = (lpn / channel_num) % chip_num;
				plane = (lpn / (channel_num*chip_num)) % plane_num;
				die = (lpn / (plane_num*channel_num*chip_num*PAGE_INDEX)) % die_num;
				break;
			}
			case CHANNEL_SUPERPAGE_STATIC_ALLOCATION:										//4.channel>chip>superpage>plane>die
			{
				channel = lpn % channel_num;
				chip = (lpn / channel_num) % chip_num;
				plane = (lpn / (channel_num*chip_num*PAGE_INDEX)) % plane_num;
				die = (lpn / (plane_num*channel_num*chip_num*PAGE_INDEX)) % die_num;
				break;
			}
			default:break;
			}
		}
		else if (ssd->parameter->flash_mode == SLC_MODE)
		{
			if (ssd->parameter->static_allocation == PLANE_STATIC_ALLOCATION || ssd->parameter->static_allocation == SUPERPAGE_STATIC_ALLOCATION)			//1.plane>channel>chip>die
			{
				plane = lpn % plane_num;
				channel = (lpn / plane_num) % channel_num;
				chip = (lpn / (plane_num*channel_num)) % chip_num;
				die = (lpn / (plane_num*channel_num*chip_num)) % die_num;
			}
			else if (ssd->parameter->static_allocation == CHANNEL_PLANE_STATIC_ALLOCATION || ssd->parameter->static_allocation == CHANNEL_SUPERPAGE_STATIC_ALLOCATION)         //2.channel>chip>plane>die
			{
				channel = lpn % channel_num;
				chip = (lpn / channel_num) % chip_num;
				plane = (lpn / (chip_num*channel_num)) % plane_num;
				die = (lpn / (plane_num*channel_num*chip_num)) % die_num;
			}
		}
	}

	//挂载到返回的结构体上
	allocated_info->channel = channel;
	allocated_info->chip = chip;
	allocated_info->die = die;
	allocated_info->plane = plane;

	return allocated_info;
}



/**********************************************
*The function is to obtain the physical 
*page number ppn for the preprocessor function
**********************************************/
unsigned int get_ppn_for_pre_process(struct ssd_info *ssd, unsigned int lpn)
{
	unsigned int channel = 0, chip = 0, die = 0, plane = 0;
	unsigned int ppn;
	unsigned int active_block;
	struct allocation_info*  lpn_location = NULL;

#ifdef DEBUG
	printf("enter get_psn_for_pre_process\n");
#endif

	//利用不同的分配算法，为预处理的数据分配位置
	lpn_location = pre_process_allocation(ssd, lpn);
	
	channel = lpn_location->channel;
	chip = lpn_location->chip;
	die = lpn_location->die;
	plane = lpn_location->plane;
	
	/******************************************************************************
	*According to the above allocation method to find channel, chip, die, plane, 
	*and then found in this active_block,Then get ppn
	******************************************************************************/
	if (find_active_block(ssd, channel, chip, die, plane) == FAILURE)
	{
		printf("the read operation is expand the capacity of SSD");
		getchar();
		return 0;
	}
	active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].active_block;
	if (write_page(ssd, channel, chip, die, plane, active_block, &ppn) == ERROR)
	{
		return 0;
	}
	
	//free掉地址空间
	free(lpn_location);
	lpn_location = NULL;
	return ppn;
}


/***************************************************************************************************
*function is given in the channel, chip, die, plane inside find an active_block and then find a page 
*inside the block, and then use find_ppn find ppn
****************************************************************************************************/
struct ssd_info *get_ppn(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, struct sub_request *sub)
{
	int old_ppn = -1;
	unsigned int ppn, lpn, full_page;
	unsigned int active_block;
	unsigned int block;
	unsigned int page, flag = 0;
	unsigned int old_state = 0, state = 0, copy_subpage = 0;
	struct local *location;
	struct direct_erase *direct_erase_node, *new_direct_erase;
	struct gc_operation *gc_node;

	unsigned int i = 0, j = 0, k = 0, l = 0, m = 0, n = 0;

#ifdef DEBUG
	printf("enter get_ppn,channel:%d, chip:%d, die:%d, plane:%d\n", channel, chip, die, plane);
#endif

	full_page = ~(0xffffffff << (ssd->parameter->subpage_page));
	lpn = sub->lpn;

	/*************************************************************************************
	*Use the function find_active_block() to find active blocks in channel, chip, die, plane
	*And modify the channel, chip, die, plane, active_block under the last_write_page and free_page_num
	**************************************************************************************/
	if (find_active_block(ssd, channel, chip, die, plane) == FAILURE)
	{
		printf("ERROR :there is no free page in channel:%d, chip:%d, die:%d, plane:%d\n", channel, chip, die, plane);
		getchar();
		return ssd;
	}

	active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].active_block;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page++;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].free_page_num--;

	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page >(ssd->parameter->page_block - 1))
	{
		printf("error! the last write page larger than max!!\n");
		while (1){}
	}

	block = active_block;
	page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page; 


	if (ssd->dram->map->map_entry[lpn].state == 0)                                       /*this is the first logical page*/
	{
		if (ssd->dram->map->map_entry[lpn].pn != 0)
		{
			printf("Error in get_ppn()\n");
			//getchar();
		}
		ssd->dram->map->map_entry[lpn].pn = find_ppn(ssd, channel, chip, die, plane, block, page);
		ssd->dram->map->map_entry[lpn].state = sub->state;
	}
	else                                                                            /*This logical page has been updated, and the original page needs to be invalidated*/
	{
		ppn = ssd->dram->map->map_entry[lpn].pn;
		location = find_location(ssd, ppn);
		if (ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].lpn != lpn)
		{

			printf("\nError in get_ppn()\n");
			//getchar();
		}

		ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].valid_state = 0;           
		ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].free_state = 0;              
		ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].lpn = 0;
		ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].invalid_page_num++;

		/*******************************************************************************************
		*The block is invalid in the page, it can directly delete, in the creation of an erase node, 
		*hanging under the location of the plane below
		********************************************************************************************/
		if (ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].invalid_page_num == ssd->parameter->page_block)
		{
			new_direct_erase = (struct direct_erase *)malloc(sizeof(struct direct_erase));
			alloc_assert(new_direct_erase, "new_direct_erase");
			memset(new_direct_erase, 0, sizeof(struct direct_erase));

			new_direct_erase->block = location->block;
			new_direct_erase->next_node = NULL;
			direct_erase_node = ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].erase_node;
			if (direct_erase_node == NULL)
			{
				ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].erase_node = new_direct_erase;
			}
			else
			{
				new_direct_erase->next_node = ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].erase_node;
				ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].erase_node = new_direct_erase;
			}
		}

		free(location);
		location = NULL;
		ssd->dram->map->map_entry[lpn].pn = find_ppn(ssd, channel, chip, die, plane, block, page);
		ssd->dram->map->map_entry[lpn].state = (ssd->dram->map->map_entry[lpn].state | sub->state);
	}


	sub->ppn = ssd->dram->map->map_entry[lpn].pn;                                      /*Modify the sub number request ppn, location and other variables*/
	sub->location->channel = channel;
	sub->location->chip = chip;
	sub->location->die = die;
	sub->location->plane = plane;
	sub->location->block = active_block;
	sub->location->page = page;

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_write_count++;
	ssd->program_count++;                                                         
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page--;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].lpn = lpn;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].valid_state = sub->state;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].free_state = ((~(sub->state))&full_page);
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].written_count++;
	ssd->write_flash_count++;

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].test_pro_count++;

	ssd->channel_head[channel].gc_soft = 0;
	ssd->channel_head[channel].gc_hard = 0;

	if (ssd->parameter->active_write == 0)
	{                                                                               /*If the number of free_page in plane is less than the threshold set by gc_hard_threshold, gc operation is generated*/
		if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page<(ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->gc_soft_threshold))
		{
			ssd->channel_head[channel].gc_soft = 1;
			if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].free_page < (ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->gc_hard_threshold))
			{
				ssd->channel_head[channel].gc_hard = 1;
			}
			gc_node = (struct gc_operation *)malloc(sizeof(struct gc_operation));
			alloc_assert(gc_node, "gc_node");
			memset(gc_node, 0, sizeof(struct gc_operation));
			if (ssd->channel_head[channel].gc_soft == 1)
				gc_node->soft = 1;
			if (ssd->channel_head[channel].gc_hard == 1)
				gc_node->hard = 1;

			gc_node->next_node = NULL;
			gc_node->channel = channel;
			gc_node->chip = chip;
			gc_node->die = die;
			gc_node->plane = plane;
			gc_node->block = 0xffffffff;
			gc_node->page = 0;
			gc_node->state = GC_WAIT;
			gc_node->priority = GC_UNINTERRUPT;
			gc_node->next_node = ssd->channel_head[channel].gc_command;
			ssd->channel_head[channel].gc_command = gc_node;
			ssd->gc_request++;
		}
	}

	return ssd;
}


/*****************************************************************************
*The function is based on the parameters channel, chip, die, plane, block, page, 
*find the physical page number
******************************************************************************/
unsigned int find_ppn(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, unsigned int block, unsigned int page)
{
	unsigned int ppn = 0;
	unsigned int i = 0;
	int page_plane = 0, page_die = 0, page_chip = 0;
	int page_channel[100];                 

#ifdef DEBUG
	printf("enter find_psn,channel:%d, chip:%d, die:%d, plane:%d, block:%d, page:%d\n", channel, chip, die, plane, block, page);
#endif

	/***************************************************************
	*Calculate the number of pages in plane, die, chip, and channel
	****************************************************************/
	page_plane = ssd->parameter->page_block*ssd->parameter->block_plane;
	page_die = page_plane*ssd->parameter->plane_die;
	page_chip = page_die*ssd->parameter->die_chip;
	while (i<ssd->parameter->channel_number)
	{
		page_channel[i] = ssd->parameter->chip_channel[i] * page_chip;
		i++;
	}

	/****************************************************************************
	*Calculate the physical page number ppn, ppn is the sum of the number of pages 
	*in channel, chip, die, plane, block, page
	*****************************************************************************/
	i = 0;
	while (i<channel)
	{
		ppn = ppn + page_channel[i];
		i++;
	}
	ppn = ppn + page_chip*chip + page_die*die + page_plane*plane + block*ssd->parameter->page_block + page;

	return ppn;
}


/************************************************************************************
*function is based on the physical page number ppn find the physical page where the 
*channel, chip, die, plane, block,In the structure location and as a return value
*************************************************************************************/
struct local *find_location(struct ssd_info *ssd, unsigned int ppn)
{
	struct local *location = NULL;
	unsigned int i = 0;
	int pn, ppn_value = ppn;
	int page_plane = 0, page_die = 0, page_chip = 0, page_channel = 0;

	pn = ppn;

#ifdef DEBUG
	printf("enter find_location\n");
#endif

	location = (struct local *)malloc(sizeof(struct local));
	alloc_assert(location, "location");
	memset(location, 0, sizeof(struct local));

	page_plane = ssd->parameter->page_block*ssd->parameter->block_plane;
	page_die = page_plane*ssd->parameter->plane_die;
	page_chip = page_die*ssd->parameter->die_chip;
	page_channel = page_chip*ssd->parameter->chip_channel[0];

	location->channel = ppn / page_channel;
	location->chip = (ppn%page_channel) / page_chip;
	location->die = ((ppn%page_channel) % page_chip) / page_die;
	location->plane = (((ppn%page_channel) % page_chip) % page_die) / page_plane;
	location->block = ((((ppn%page_channel) % page_chip) % page_die) % page_plane) / ssd->parameter->page_block;
	location->page = (((((ppn%page_channel) % page_chip) % page_die) % page_plane) % ssd->parameter->page_block) % ssd->parameter->page_block;

	return location;
}

/*******************************************************************
*When executing a write request, get ppn for a normal write request
*********************************************************************/
Status get_ppn_for_normal_command(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request * sub)
{
	unsigned int die, plane;

	if (sub == NULL)
	{
		return ERROR;
	}
	if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION || ssd->parameter->allocation_scheme == HYBRID_ALLOCATION)
	{
		die = ssd->channel_head[channel].chip_head[chip].token;
		plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
		get_ppn(ssd, channel, chip, die, plane, sub);

		//更新下次操作的令牌
		if (ssd->parameter->dynamic_allocation == PLANE_DYNAMIC_ALLOCATION)
		{
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
			if (plane == (ssd->parameter->plane_die - 1))
				ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
		}
		else if (ssd->parameter->dynamic_allocation == STRIPE_DYNAMIC_ALLOCATION || ssd->parameter->dynamic_allocation == OSPA_DYNAMIC_ALLOCATION || ssd->parameter->dynamic_allocation == POLL_DISTRANCE_ALLOCATION)
		{
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
			if (ssd->channel_head[channel].chip_head[chip].die_head[die].token == 0)
			{
				ssd->channel_head[channel].token = (ssd->channel_head[channel].token + 1) % ssd->parameter->chip_channel[0];
				ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = (die + 1) % ssd->parameter->die_chip;
				if (ssd->channel_head[channel].token == 0)
					ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;
			}
		}
		else if (ssd->parameter->dynamic_allocation == CHANNEL_DYNAMIC_ALLOCATION)
		{
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
			ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
		}
		compute_serve_time(ssd, channel, chip, die, &sub, 1, NORMAL);
	}
	else if (ssd->parameter->allocation_scheme == STATIC_ALLOCATION)
	{
		die = sub->location->die;
		if (ssd->parameter->flash_mode == TLC_MODE)
			sub->location->plane = 0;

		plane = sub->location->plane;
		get_ppn(ssd, channel, chip, die, plane, sub);

		compute_serve_time(ssd, channel, chip, die, &sub, 1, NORMAL);
	}

	return SUCCESS;
}

/************************************************************************************************
*Write a request for an advanced command to get ppn
*According to different orders, in accordance with the same block in the order to write the request, 
*select the write can be done ppn, skip the ppn all set to invaild
*
*In the use of two plane operation, in order to find the same level of the page, you may need to 
*directly find two completely blank block, this time the original block is not used up, can only be 
*placed on this, waiting for the next use, while modifying the search blank Page method, will be the 
*first to find free block to change, as long as the invalid block! = 64 can.
*
*except find aim page, we should modify token and decide gc operation
*************************************************************************************************/
Status get_ppn_for_advanced_commands(struct ssd_info *ssd, unsigned int channel, unsigned int chip, struct sub_request ** subs, unsigned int subs_count, unsigned int command)
{
	unsigned int aim_die = 0, aim_plane = 0, aim_count = 0;
	unsigned int die_token = 0, plane_token = 0;
	unsigned int i = 0, j = 0, k = 0;
	unsigned int valid_subs_count = 0;
	unsigned int state ;

	struct sub_request * sub = NULL;
	struct sub_request ** mutli_subs = NULL;

	if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION || ssd->parameter->allocation_scheme == HYBRID_ALLOCATION)  //动态分配的目标die plane由动态令牌来决定
	{
		aim_die = ssd->channel_head[channel].chip_head[chip].token;
		aim_plane = ssd->channel_head[channel].chip_head[chip].die_head[aim_die].token;
	}
	else if (ssd->parameter->allocation_scheme == STATIC_ALLOCATION)
	{
		aim_die = subs[0]->location->die;            //静态分配只能找到对应的die
		//验证subs的有效性
		for (i = 0; i < subs_count; i++)
		{
			if (subs[i]->location->die != aim_die)
			{
				printf("Error ,aim_die match failed\n");
				getchar();
			}
		}
	}

	//如果是one shot mutli plane的情况，这里就要分superpage还是mutli plane优先
	if (command == ONE_SHOT_MUTLI_PLANE)
	{
		mutli_subs = (struct sub_request **)malloc(ssd->parameter->plane_die * sizeof(struct sub_request *));
		//plane>superpage:024/135
		if (ssd->parameter->static_allocation == PLANE_STATIC_ALLOCATION || ssd->parameter->static_allocation == CHANNEL_PLANE_STATIC_ALLOCATION )
		{
			for (i = 0; i < PAGE_INDEX; i++)
			{
				for (j = 0; j < ssd->parameter->plane_die; j++)
				{
					if (i + k > subs_count)
					{
						printf("subs_count distribute error\n");
						getchar();
					}
					mutli_subs[j] = subs[j + k];
				}
				//进行mutli plane的操作
				find_level_page(ssd, channel, chip, aim_die, mutli_subs, ssd->parameter->plane_die);
				k = k + ssd->parameter->plane_die;
			}
		}//superpage>plane;012/345
		else if (ssd->parameter->static_allocation == SUPERPAGE_STATIC_ALLOCATION || ssd->parameter->static_allocation == CHANNEL_SUPERPAGE_STATIC_ALLOCATION)
		{
			for (i = 0; i < PAGE_INDEX; i++)
			{
				k = 0;
				for (j = 0; j < ssd->parameter->plane_die; j++)
				{
					if (i + k > subs_count)
					{
						printf("subs_count distribute error\n");
						getchar();
					}
					mutli_subs[j] = subs[i + k];
					k = k + PAGE_INDEX;
				}
				//进行mutli plane的操作
				find_level_page(ssd, channel, chip, aim_die, mutli_subs, ssd->parameter->plane_die);
			}
		}

		valid_subs_count = subs_count;
		compute_serve_time(ssd, channel, chip, aim_die, subs, valid_subs_count, ONE_SHOT_MUTLI_PLANE);
		//printf("lz:mutli plane one shot\n");

		if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION || ssd->parameter->allocation_scheme == HYBRID_ALLOCATION)
			ssd->channel_head[channel].chip_head[chip].token = (aim_die + 1) % ssd->parameter->die_chip;

		//free mutli_subs
		for (i = 0; i < ssd->parameter->plane_die; i++)
			mutli_subs[i] = NULL;
		free(mutli_subs);
		mutli_subs = NULL;
		return SUCCESS;
	}
	else if (command == MUTLI_PLANE)
	{
		if (subs_count == ssd->parameter->plane_die)
		{
			state = find_level_page(ssd, channel, chip, aim_die, subs, subs_count);
			if (state != SUCCESS)
			{
				get_ppn_for_normal_command(ssd, channel, chip, subs[0]);		 
				printf("find_level_page failed, begin to one page program\n");
				getchar();
				return FAILURE;
			}
			else
			{
				valid_subs_count = ssd->parameter->plane_die;
				compute_serve_time(ssd, channel, chip, aim_die, subs, valid_subs_count, MUTLI_PLANE);

				if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION || ssd->parameter->allocation_scheme == HYBRID_ALLOCATION)
					ssd->channel_head[channel].chip_head[chip].token = (aim_die + 1) % ssd->parameter->die_chip;
				
				//printf("lz:mutli_plane\n");
				return SUCCESS;
			}
		}
		else
		{
			return ERROR;
		}
	}
	else if (command == ONE_SHOT)
	{
		for (i = 0; i < subs_count; i++)
			get_ppn(ssd, channel, chip, aim_die, aim_plane, subs[i]);

		valid_subs_count = PAGE_INDEX;
		compute_serve_time(ssd, channel, chip, aim_die, subs, valid_subs_count, ONE_SHOT);

		//更新plane die
		if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION || ssd->parameter->allocation_scheme == HYBRID_ALLOCATION)
		{
			ssd->channel_head[channel].chip_head[chip].die_head[aim_die].token = (aim_plane + 1) % ssd->parameter->plane_die;
			if (aim_plane == (ssd->parameter->plane_die - 1))
				ssd->channel_head[channel].chip_head[chip].token = (aim_die + 1) % ssd->parameter->die_chip;
		}

		//printf("lz:one shot\n");
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}

/******************************************************************************************下面是ftl层gc操作******************************************************************************************/

/************************************************************************************************************
*Gc operation, for the invalid block, the use of mutli erase select two plane offset address of the same invalid block to erase,
*For the valid block, select the two planes within the invalid page of the largest block to erase, and migrate a valid page, 
*the purpose of this is to ensure that the use of mutli hit, that is, for the die, each erase the super block , In the mutli 
*plane write, only need to ensure that the page offset consistent, do not guarantee blcok offset address can be consistent.
************************************************************************************************************/

void gc_check(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int old_plane)
{
	struct gc_operation *gc_node;
	unsigned int i;

	ssd->channel_head[channel].gc_soft = 0;
	ssd->channel_head[channel].gc_hard = 0;
	for (i = 0; i < ssd->parameter->plane_die; i++)
	{
		if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].free_page <= (ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->gc_soft_threshold))
		{
			ssd->channel_head[channel].gc_soft = 1;
			if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].free_page <= (ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->gc_hard_threshold))
			{
				ssd->channel_head[channel].gc_hard = 1;
			}
		}
	}
	if (ssd->channel_head[channel].gc_soft == 1)		//produce a gc reqeuest and add gc_node to the channel
	{
		gc_node = (struct gc_operation *)malloc(sizeof(struct gc_operation));
		alloc_assert(gc_node, "gc_node");
		memset(gc_node, 0, sizeof(struct gc_operation));

		gc_node->next_node = NULL;
		gc_node->channel = channel;
		gc_node->chip = chip;
		gc_node->die = die;
		gc_node->plane = old_plane;
		gc_node->block = 0xffffffff;
		gc_node->page = 0;
		gc_node->state = GC_WAIT;
		gc_node->priority = GC_UNINTERRUPT;
		gc_node->next_node = ssd->channel_head[channel].gc_command;
		ssd->channel_head[channel].gc_command = gc_node;					//inserted into the head of the gc chain
		ssd->gc_request++;
	}
}


unsigned int gc(struct ssd_info *ssd, unsigned int channel, unsigned int flag)
{
	unsigned int i;

	//printf("gc flag=%d\n",flag);
	//Active gc
	if (flag == 1)                                                                       /*The whole ssd is the case of IDEL*/
	{
		for (i = 0; i<ssd->parameter->channel_number; i++)
		{
			if ((ssd->channel_head[i].current_state == CHANNEL_IDLE) || (ssd->channel_head[i].next_state == CHANNEL_IDLE&&ssd->channel_head[i].next_state_predict_time <= ssd->current_time))
			{
				if (ssd->channel_head[i].gc_command != NULL)
				{
					if (gc_for_channel(ssd, i, 1) == SUCCESS)
					{
						ssd->gc_count++;
					}
				}
			}
		}
		return SUCCESS;
	}
	//Passive gc
	else
	{
		//当读写子请求都完成的情况下，才去执行gc操作，否则先去执行读写请求
		//if ((ssd->channel_head[channel].subs_r_head != NULL) || (ssd->channel_head[channel].subs_w_head != NULL) || (ssd->subs_w_head != NULL))    
		//{
		//return 0;
		//}
		if (gc_for_channel(ssd, channel, 0) == SUCCESS)
		{
			ssd->gc_count++;
			return SUCCESS;
		}
		else
			return FAILURE;
	}
	return FAILURE;
}


/************************************************************
*this function is to handle every gc operation of the channel
************************************************************/
Status gc_for_channel(struct ssd_info *ssd, unsigned int channel, unsigned int flag)
{
	int flag_direct_erase = 1, flag_gc = 1, flag_suspend = 1;
	unsigned int chip, die, plane, flag_priority = 0;
	unsigned int hard, soft;
	struct gc_operation *gc_node = NULL;

	/*******************************************************************************************
	*Find each gc_node, get the current state of the chip where gc_node is located, the next state,
	*the expected time of the next state .If the current state is idle, or the next state is idle
	*and the next state is expected to be less than the current time, and is not interrupted gc
	*Then the flag_priority order is 1, otherwise 0.
	********************************************************************************************/
	gc_node = ssd->channel_head[channel].gc_command;
	while (gc_node != NULL)
	{
		//如果当前chip发生了suspend操作，此时整个chip表示busy状态
		if (ssd->channel_head[channel].chip_head[gc_node->chip].gc_signal != SIG_NORMAL)
			flag_suspend = 0;
		else
			flag_suspend = 1;

		if (flag_suspend == 1)
		{
			if ((ssd->channel_head[channel].chip_head[gc_node->chip].current_state == CHIP_IDLE) ||
				((ssd->channel_head[channel].chip_head[gc_node->chip].next_state == CHIP_IDLE) && (ssd->channel_head[channel].chip_head[gc_node->chip].next_state_predict_time <= ssd->current_time)))
			{
				if (gc_node->priority == GC_UNINTERRUPT)                                     /*this gc request is not interrupted, the priority service gc operation*/
				{
					flag_priority = 1;
					break;																	/*Processing the nearest free node on the current channel gc request chain*/
				}
			}
		}
		gc_node = gc_node->next_node;
	}

	if (gc_node == NULL)
	{
		return FAILURE;
	}

	chip = gc_node->chip;
	die = gc_node->die;
	plane = gc_node->plane;
	//hard = gc_node->hard;
	//soft = gc_node->soft;
	if (flag == 0)
	{
		if (gc_node->priority == GC_UNINTERRUPT /*&& hard == 1 && soft == 1*/)
		{
			flag_direct_erase = gc_direct_erase(ssd, channel, chip, die);
			if (flag_direct_erase != SUCCESS)
			{
				flag_gc = greedy_gc(ssd, channel, chip, die);							 /*When a complete gc operation is completed, return 1, the corresponding channel gc operation request node to delete*/
				if (flag_gc == SUCCESS)
				{
					delete_gc_node(ssd, channel, gc_node);
				}
				else
				{
					return FAILURE;
				}

			}
			else
			{
				delete_gc_node(ssd, channel, gc_node);
			}
			return SUCCESS;
		}
	}
	else if (flag == 1)
	{
		if (gc_node->priority == GC_UNINTERRUPT)
		{
			flag_direct_erase = gc_direct_erase(ssd, channel, chip, die);
			if (flag_direct_erase != SUCCESS)
			{
				flag_gc = greedy_gc(ssd, channel, chip, die);							 /*When a complete gc operation is completed, return 1, the corresponding channel gc operation request node to delete*/
				if (flag_gc == SUCCESS)
				{
					delete_gc_node(ssd, channel, gc_node);
				}
				else
				{
					return FAILURE;
				}

			}
			else
			{
				delete_gc_node(ssd, channel, gc_node);
			}
			return SUCCESS;
		}
	}

	return FAILURE;
}



/*******************************************************************************************************************
*GC operation in a number of plane selected two offset address of the same block to erase, and in the invalid block 
*on the table where the invalid block node, erase success, calculate the mutli plane erase operation of the implementation 
*time, channel chip status Change time
*********************************************************************************************************************/
int gc_direct_erase(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die)
{
	unsigned int i,j, plane, block;		
	unsigned int * erase_block;
	struct direct_erase * direct_erase_node = NULL;

	erase_block = (unsigned int*)malloc(ssd->parameter->plane_die * sizeof(erase_block));
	for ( i = 0; i < ssd->parameter->plane_die; i++)
	{
		direct_erase_node = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].erase_node;
		if (direct_erase_node == NULL)
		{
			free(erase_block);
			erase_block = NULL;
			return FAILURE;
		}

		//Perform mutli plane erase operation,and delete gc_node
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].erase_node = direct_erase_node->next_node;
		erase_block[i] = direct_erase_node->block;

		free(direct_erase_node);
		ssd->direct_erase_count++;
		direct_erase_node = NULL;
	}

	//首先进行channel的跳转，仅是传输命令的时间
	ssd->channel_head[channel].current_state = CHANNEL_TRANSFER;
	ssd->channel_head[channel].current_time = ssd->current_time;
	ssd->channel_head[channel].next_state = CHANNEL_IDLE;
	ssd->channel_head[channel].next_state_predict_time = ssd->current_time + 7 * ssd->parameter->plane_die * ssd->parameter->time_characteristics.tWC;   //14表示的是传输命令的时间，为mutli plane


	//判断是否有suspend命令，有次命令则不能擦除，要挂起擦除操作，等待恢复
	if ((ssd->parameter->advanced_commands&AD_ERASE_SUSPEND_RESUME) == AD_ERASE_SUSPEND_RESUME)
	{
		//1.使用了suspend命令，首先更改chip上的suspend请求状态，用于检测是否有suspend请求到来
		ssd->channel_head[channel].chip_head[chip].gc_signal = SIG_ERASE_WAIT;
		ssd->channel_head[channel].chip_head[chip].erase_begin_time = ssd->channel_head[channel].next_state_predict_time;
		ssd->channel_head[channel].chip_head[chip].erase_cmplt_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tBERS + ssd->parameter->time_characteristics.tERSL;

		//2.保留擦除操作的现场,产生一个suspend_erase_command请求挂载在chip上
		suspend_erase_operation(ssd, channel, chip, die, erase_block);
	}
	else
	{
		for (j = 0; j < ssd->parameter->plane_die; j++)
		{
			plane = j;
			block = erase_block[j];
			erase_operation(ssd, channel, chip, die, plane, block);
		}

		ssd->mplane_erase_count++;
		ssd->channel_head[channel].chip_head[chip].current_state = CHIP_ERASE_BUSY;
		ssd->channel_head[channel].chip_head[chip].current_time = ssd->current_time;
		ssd->channel_head[channel].chip_head[chip].next_state = CHIP_IDLE;
		ssd->channel_head[channel].chip_head[chip].next_state_predict_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tBERS;
	}
	free(erase_block);
	erase_block = NULL;
	return SUCCESS;
}

/*******************************************************************************************************************************************
*The target plane can not be directly deleted by the block, need to find the target erase block after the implementation of the erase operation, 
*the successful deletion of a block, returns 1, does not delete a block returns -1
********************************************************************************************************************************************/
int greedy_gc(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die)
{
	unsigned int i = 0, j = 0, p = 0 ,invalid_page = 0;
	unsigned int active_block1, active_block2, transfer_size = 0, free_page, avg_page_move = 0;                           /*Record the maximum number of blocks that are invalid*/
	struct local *  location = NULL;
	unsigned int plane , move_plane;
	int block1, block2;
	
	unsigned int active_block;
	unsigned int block;
	unsigned int page_move_count = 0;
	struct direct_erase * direct_erase_node_tmp = NULL;
	struct direct_erase * pre_erase_node_tmp = NULL;
	unsigned int * erase_block;
	unsigned int aim_page;


	erase_block = (unsigned int*)malloc( ssd->parameter->plane_die * sizeof(erase_block));
	//gets active blocks within all plane
	for ( p = 0; p < ssd->parameter->plane_die; p++)
	{
		if ( find_active_block(ssd, channel, chip, die, p) != SUCCESS )
		{
			free(erase_block);
			erase_block = NULL;
			printf("\n\n Error in uninterrupt_gc().\n");
			return ERROR;
		}
		active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[p].active_block;
		
		//find the largest number of invalid pages in plane
		invalid_page = 0;
		block = -1;
		direct_erase_node_tmp = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[p].erase_node;
		for (i = 0; i<ssd->parameter->block_plane; i++)																					 /*Find the maximum number of invalid_page blocks, and the largest invalid_page_num*/
		{
			if ((active_block != i) && (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[p].blk_head[i].invalid_page_num>invalid_page)) /*Can not find the current active block*/
			{
				invalid_page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[p].blk_head[i].invalid_page_num;
				block = i;
			}
		}
		//Check whether all is invalid page, if all is, then the current block is invalid block, need to remove this node from the erase chain
		if (invalid_page == ssd->parameter->page_block)
		{
			while (direct_erase_node_tmp != NULL)
			{
				if (block == direct_erase_node_tmp->block)
				{
					if (pre_erase_node_tmp == NULL)
						ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[p].erase_node = direct_erase_node_tmp->next_node;
					else
						pre_erase_node_tmp->next_node = direct_erase_node_tmp->next_node;

					free(direct_erase_node_tmp);
					direct_erase_node_tmp = NULL;
					break;
				}
				else
				{
					pre_erase_node_tmp = direct_erase_node_tmp;
					direct_erase_node_tmp = direct_erase_node_tmp->next_node;
				}
			}
			pre_erase_node_tmp = NULL;
			direct_erase_node_tmp = NULL;
		}

		//Found the block to be erased
		if (block == -1)
		{
			free(erase_block);
			erase_block = NULL;
			return ERROR;
		}

		//caculate sum of  vaild page_move count
		page_move_count += ssd->parameter->page_block - ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[p].blk_head[block].invalid_page_num;
		erase_block[p] = block;
	}

	//caculate the average of the sum vaild page_block,and distribute equally to all plane of die
	avg_page_move = page_move_count / (ssd->parameter->plane_die);

	//Perform a migration of valid data pages
	free_page = 0;
	page_move_count = 0;
	move_plane = 0;
	for (j = 0; j < ssd->parameter->plane_die; j++)
	{
		plane = j;
		block = erase_block[j];
		for (i = 0; i < ssd->parameter->page_block; i++)		                                                     /*Check each page one by one, if there is a valid data page need to move to other places to store*/
		{
			if ((ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[i].free_state&PG_SUB) == 0x0000000f)
			{
				free_page++;
			}
			if (free_page != 0)
			{
				printf("\ntoo much free page. \t %d\t .%d\t%d\t%d\t%d\t\n", free_page, channel, chip, die, plane); /*There are free pages, proved to be active blocks, blocks are not finished, can not be erased*/
				//getchar();
			}

			if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[i].valid_state > 0) /*The page is a valid page that requires a copyback operation*/
			{
				location = (struct local *)malloc(sizeof(struct local));
				alloc_assert(location, "location");
				memset(location, 0, sizeof(struct local));
				location->channel = channel;
				location->chip = chip;
				location->die = die;
				location->plane = plane;
				location->block = block;
				location->page = i;
				page_move_count++;

				move_page(ssd, location, move_plane, &transfer_size);                                                   /*Real move_page operation*/
				move_plane = (move_plane + 1) % ssd->parameter->plane_die;

				free(location);
				location = NULL;
			}
		}
	}

	//当move plane不等0的时候，表示此时是单数，需要磨平
	if (move_plane != 0)
	{
		find_active_block(ssd, channel, chip, die, move_plane);
		active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[move_plane].active_block;
		aim_page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[move_plane].blk_head[active_block].last_write_page + 2;
		if (aim_page == ssd->parameter->page_block + 1)
			getchar();
		make_same_level(ssd, channel, chip, die, move_plane, active_block, aim_page);
	}
	
	//判断是否偏移地址free page一致
	/*
	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[0].free_page != ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[1].free_page)
	{
		printf("free page don't equal\n");
		getchar();
	}*/

	//迁移有效页的时间推动
	ssd->channel_head[channel].current_state = CHANNEL_GC;
	ssd->channel_head[channel].current_time = ssd->current_time;
	ssd->channel_head[channel].next_state = CHANNEL_IDLE;
	ssd->channel_head[channel].next_state_predict_time = ssd->current_time +
	page_move_count*(7 * ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tR + 7 * ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tPROG) +
	transfer_size*SECTOR*(ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tRC);

	//有效页迁移完成，开始执行擦除操作,擦除两个block
	if ((ssd->parameter->advanced_commands&AD_ERASE_SUSPEND_RESUME) == AD_ERASE_SUSPEND_RESUME)
	{
		//1.使用了suspend命令，首先更改chip上的suspend请求状态，用于检测是否有suspend请求到来
		ssd->channel_head[channel].chip_head[chip].gc_signal = SIG_ERASE_WAIT;
		ssd->channel_head[channel].chip_head[chip].erase_begin_time = ssd->channel_head[channel].next_state_predict_time;
		ssd->channel_head[channel].chip_head[chip].erase_cmplt_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tBERS + ssd->parameter->time_characteristics.tERSL;

		//2.保留擦除操作的现场,产生一个suspend_erase_command请求挂载在chip上
		suspend_erase_operation(ssd, channel, chip, die, erase_block);
	}
	else
	{
		for (j = 0; j < ssd->parameter->plane_die; j++)
		{
			plane = j;
			block = erase_block[j];
			erase_operation(ssd, channel, chip, die, plane, block);					
		}
		ssd->mplane_erase_count++;
		ssd->channel_head[channel].chip_head[chip].current_state = CHIP_ERASE_BUSY;
		ssd->channel_head[channel].chip_head[chip].current_time = ssd->current_time;
		ssd->channel_head[channel].chip_head[chip].next_state = CHIP_IDLE;
		ssd->channel_head[channel].chip_head[chip].next_state_predict_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tBERS;
	}
	
	free(erase_block);
	erase_block = NULL;
	return SUCCESS;
}

//执行suspend挂起操作，保留擦写操作的现场
struct ssd_info * suspend_erase_operation(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int * erase_block)
{
	long long erase_begin_time, erase_end_time;
	unsigned int flag = 0, j = 0;
	struct suspend_location * location = NULL;

	//1.保留现场，suspend擦除操作的开始时间、结束时间
	erase_begin_time = ssd->channel_head[channel].next_state_predict_time;
	erase_end_time = erase_begin_time + ssd->parameter->time_characteristics.tBERS + ssd->parameter->time_characteristics.tERSL;
	
	//2.保护现场，产生一个suspend的location地址，并挂载在chip上
	location = (struct suspend_location *)malloc(sizeof(struct suspend_location));
	alloc_assert(location, "location");
	memset(location, 0, sizeof(struct suspend_location));

	location->channel = channel;
	location->chip = chip;
	location->die = die;
	for (j = 0; j < ssd->parameter->plane_die; j++)
	{
		location->plane[j] = j;
		location->block[j] = erase_block[j];
	}

	//3.挂载chip上
	ssd->channel_head[channel].chip_head[chip].suspend_location = location;
	return ssd;
}

Status resume_erase_operation(struct ssd_info * ssd, unsigned int channel, unsigned int chip)
{
	unsigned int j = 0;
	struct suspend_location * resume_location = NULL;
	struct sub_request * sub = NULL ,* pre_sub = NULL;

	resume_location = ssd->channel_head[channel].chip_head[chip].suspend_location;
	if (resume_location != NULL)
	{
		//3.重置erase的状态时间线
		ssd->channel_head[resume_location->channel].chip_head[resume_location->chip].gc_signal = SIG_NORMAL;
		ssd->channel_head[resume_location->channel].chip_head[resume_location->chip].erase_begin_time = 0;
		ssd->channel_head[resume_location->channel].chip_head[resume_location->chip].erase_cmplt_time = 0;
		ssd->channel_head[resume_location->channel].chip_head[resume_location->chip].erase_rest_time = 0;

		//4.执行擦除操作
		for (j = 0; j < ssd->parameter->plane_die; j++)
			erase_operation(ssd, resume_location->channel, resume_location->chip, resume_location->die, resume_location->plane[j], resume_location->block[j]);

		ssd->mplane_erase_count++;
		free(resume_location);
		ssd->channel_head[channel].chip_head[chip].suspend_location = NULL;
		return SUCCESS;
	}
	else
	{
		printf("resume failed\n");
		getchar();
		return FAILURE;
	}
}


/*****************************************************************************************
*This function is for the gc operation to find a new ppn, because in the gc operation need 
*to find a new physical block to store the original physical block data
******************************************************************************************/
unsigned int get_ppn_for_gc(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane)
{
	unsigned int ppn;
	unsigned int active_block, block, page;

#ifdef DEBUG
	printf("enter get_ppn_for_gc,channel:%d, chip:%d, die:%d, plane:%d\n", channel, chip, die, plane);
#endif

	if (find_active_block(ssd, channel, chip, die, plane) != SUCCESS)
	{
		printf("\n\n Error int get_ppn_for_gc().\n");
		return 0xffffffff;
	}

	active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].active_block;

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page++;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].free_page_num--;

	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page>(ssd->parameter->page_block - 1))
	{
		printf("error! the last write page larger than max!!\n");
		while (1){}
	}

	block = active_block;
	page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page;

	ppn = find_ppn(ssd, channel, chip, die, plane, block, page);

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page--;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_write_count++;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].written_count++;
	ssd->write_flash_count++;
	ssd->program_count++;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].test_gc_count++;

	return ppn;

}
/*************************************************************
*function is when dealing with a gc operation, the need to gc 
*chain gc_node deleted
**************************************************************/
int delete_gc_node(struct ssd_info *ssd, unsigned int channel, struct gc_operation *gc_node)
{
	struct gc_operation *gc_pre = NULL;
	if (gc_node == NULL)
	{
		return ERROR;
	}

	if (gc_node == ssd->channel_head[channel].gc_command)
	{
		ssd->channel_head[channel].gc_command = gc_node->next_node;
	}
	else
	{
		gc_pre = ssd->channel_head[channel].gc_command;
		while (gc_pre->next_node != NULL)
		{
			if (gc_pre->next_node == gc_node)
			{
				gc_pre->next_node = gc_node->next_node;
				break;
			}
			gc_pre = gc_pre->next_node;
		}
	}
	free(gc_node);
	gc_node = NULL;
	ssd->gc_request--;
	return SUCCESS;
}


/**************************************************************************************
*Function function is to find active fast, there should be only one active block for 
*each plane, only the active block in order to operate
***************************************************************************************/
Status  find_active_block(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane)
{
	unsigned int active_block = 0;
	unsigned int free_page_num = 0;
	unsigned int count = 0;
	//	int i, j, k, p, t;

	active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].active_block;
	free_page_num = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].free_page_num;
	//last_write_page=ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].free_page_num;
	while ((free_page_num == 0) && (count<ssd->parameter->block_plane))
	{
		active_block = (active_block + 1) % ssd->parameter->block_plane;
		free_page_num = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].free_page_num;
		count++;
	}

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].active_block = active_block;

	if (count<ssd->parameter->block_plane)
	{
		return SUCCESS;
	}
	else
	{
		return FAILURE;
	}
}


