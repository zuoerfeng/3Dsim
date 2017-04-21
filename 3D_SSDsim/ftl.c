/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName： ssd.c
Author: Zuo Lu 		Version: 1.0	Date:2017/04/06
Description: 
ftl layer: can not interrupt the global gc operation, gc operation to migrate valid pages using ordinary read and write operations, remove support copyback operation;

History:
<contributor>     <time>        <version>       <desc>                   <e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim       617376665@qq.com

*****************************************************************************************************************************/

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

extern int plane_cmplt;

/******************************************************************************************下面是ftl层map操作******************************************************************************************/

/**************************************************
*读请求预处理函数，当读请求所读得页里面没有数据时，
*需要预处理网该页里面写数据，以保证能读到数据
***************************************************/
struct ssd_info *pre_process_page(struct ssd_info *ssd)
{
	int fl = 0;
	unsigned int device, lsn, size, ope, lpn, full_page;
	unsigned int largest_lsn, sub_size, ppn, add_size = 0;
	unsigned int i = 0, j, k, p;
	int map_entry_new, map_entry_old, modify;
	int flag = 0;
	char buffer_request[200];
	struct local *location;
	long double time0;
	//unsigned long time1;
	__int64 time;
	errno_t err;


	printf("\n");
	printf("begin pre_process_page.................\n");

	if ((err = fopen_s(&(ssd->tracefile), ssd->tracefilename, "r")) != 0)      /*打开trace文件从中读取请求*/
	{
		printf("the trace file can't open\n");
		return NULL;
	}

	full_page = ~(0xffffffff << (ssd->parameter->subpage_page));
	/*计算出这个ssd的最大逻辑扇区号*/
	largest_lsn = (unsigned int)((ssd->parameter->chip_num*ssd->parameter->die_chip*ssd->parameter->plane_die*ssd->parameter->block_plane*ssd->parameter->page_block*ssd->parameter->subpage_page)*(1 - ssd->parameter->overprovide));

	while (fgets(buffer_request, 200, ssd->tracefile))
	{
		//sscanf_s(buffer_request,"%Lf %d %d %d %d",&time0,&device,&lsn,&size,&ope);
		//time_count++;
		//printf("count=%d\n", time_count);
		//time1 = (unsigned long)(time0 * 100000.0);
		//time = (__int64)(time0 * 1000000.0);

		sscanf_s(buffer_request, "%I64u %d %d %d %d", &time, &device, &lsn, &size, &ope);
		fl++;
		trace_assert(time, device, lsn, size, ope);                         /*断言，当读到的time，device，lsn，size，ope不合法时就会处理*/

		add_size = 0;                                                     /*add_size是这个请求已经预处理的大小*/

		if (ope == 1)                                                      /*这里只是读请求的预处理，需要提前将相应位置的信息进行相应修改*/
		{
			while (add_size<size)
			{
				lsn = lsn%largest_lsn;                                    /*防止获得的lsn比最大的lsn还大*/
				sub_size = ssd->parameter->subpage_page - (lsn%ssd->parameter->subpage_page);
				if (add_size + sub_size >= size)                             /*只有当一个请求的大小小于一个page的大小时或者是处理一个请求的最后一个page时会出现这种情况*/
				{
					sub_size = size - add_size;
					add_size += sub_size;
				}

				if ((sub_size>ssd->parameter->subpage_page) || (add_size>size))/*当预处理一个子大小时，这个大小大于一个page或是已经处理的大小大于size就报错*/
				{
					printf("pre_process sub_size:%d\n", sub_size);
				}

				/*******************************************************************************************************
				*利用逻辑扇区号lsn计算出逻辑页号lpn
				*判断这个dram中映射表map中在lpn位置的状态
				*A，这个状态==0，表示以前没有写过，现在需要直接将ub_size大小的子页写进去写进去
				*B，这个状态>0，表示，以前有写过，这需要进一步比较状态，因为新写的状态可以与以前的状态有重叠的扇区的地方
				********************************************************************************************************/
				lpn = lsn / ssd->parameter->subpage_page;
				if (ssd->dram->map->map_entry[lpn].state == 0)                 /*状态为0的情况*/
				{
					/**************************************************************
					*获得利用get_ppn_for_pre_process函数获得ppn，再得到location
					*修改ssd的相关参数，dram的映射表map，以及location下的page的状态
					***************************************************************/
					ppn = get_ppn_for_pre_process(ssd, lsn);
					location = find_location(ssd, ppn);

					//ssd->program_count++;
					//ssd->channel_head[location->channel].program_count++;
					//ssd->channel_head[location->channel].chip_head[location->chip].program_count++;
					ssd->pre_all_write++;
					ssd->dram->map->map_entry[lpn].pn = ppn;
					ssd->dram->map->map_entry[lpn].state = set_entry_state(ssd, lsn, sub_size);   //0001
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].pre_write_count++;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].lpn = lpn;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].valid_state = ssd->dram->map->map_entry[lpn].state;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].free_state = ((~ssd->dram->map->map_entry[lpn].state)&full_page);
					
					free(location);
					location = NULL;
				}//if(ssd->dram->map->map_entry[lpn].state==0)
				else if (ssd->dram->map->map_entry[lpn].state>0)           /*状态不为0的情况*/
				{
					map_entry_new = set_entry_state(ssd, lsn, sub_size);      /*得到新的状态，并与原来的状态相或的到一个状态*/
					map_entry_old = ssd->dram->map->map_entry[lpn].state;
					modify = map_entry_new | map_entry_old;
					ppn = ssd->dram->map->map_entry[lpn].pn;				/*这里没有必要把最新的状态刷到闪存上去，因为此时表已经建立起来了，表里面记录的状态最新的即可*/
					location = find_location(ssd, ppn);

					//ssd->program_count++;
					//ssd->channel_head[location->channel].program_count++;
					//ssd->channel_head[location->channel].chip_head[location->chip].program_count++;
					//ssd->pre_all_write++;
					ssd->dram->map->map_entry[lsn / ssd->parameter->subpage_page].state = modify;
					//ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].pre_write_count++;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].valid_state = modify;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].free_state = ((~modify)&full_page);

					free(location);
					location = NULL;
				}//else if(ssd->dram->map->map_entry[lpn].state>0)
				lsn = lsn + sub_size;                                         /*下个子请求的起始位置*/
				add_size += sub_size;                                       /*已经处理了的add_size大小变化*/
			}//while(add_size<size)
		}//if(ope==1) 
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


/********************************
*函数功能是获得一个读子请求的状态
*********************************/
int set_entry_state(struct ssd_info *ssd, unsigned int lsn, unsigned int size)
{
	int temp, state, move;

	temp = ~(0xffffffff << size);
	move = lsn%ssd->parameter->subpage_page;
	state = temp << move;

	return state;
}

/**************************************
*函数功能是为预处理函数获取物理页号ppn
*获取页号分为动态获取和静态获取
**************************************/
unsigned int get_ppn_for_pre_process(struct ssd_info *ssd, unsigned int lsn)
{
	unsigned int channel = 0, chip = 0, die = 0, plane = 0;
	unsigned int ppn, lpn;
	unsigned int active_block;
	unsigned int channel_num = 0, chip_num = 0, die_num = 0, plane_num = 0;

#ifdef DEBUG
	printf("enter get_psn_for_pre_process\n");
#endif

	channel_num = ssd->parameter->channel_number;
	chip_num = ssd->parameter->chip_channel[0];
	die_num = ssd->parameter->die_chip;
	plane_num = ssd->parameter->plane_die;
	lpn = lsn / ssd->parameter->subpage_page;

	if (ssd->parameter->allocation_scheme == 0)                           /*动态方式下获取ppn*/
	{
		//这里的优先级：channel>die>plane
		/*
		if (ssd->parameter->dynamic_allocation == 0)                      //*表示全动态方式下，也就是channel，chip，die，plane，block等都是动态分配
		{
			channel = ssd->token;
			ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;
			chip = ssd->channel_head[channel].token;
			ssd->channel_head[channel].token = (chip + 1) % ssd->parameter->chip_channel[0];
			die = ssd->channel_head[channel].chip_head[chip].token;
			ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
			plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
		}
		*/
		
		//重新修改优先级：plane>channel>die
		if (ssd->parameter->dynamic_allocation == 0)                      /*表示全动态方式下，也就是channel，chip，die，plane，block等都是动态分配*/
		{
			channel = ssd->token;
			chip = ssd->channel_head[channel].token;
			die = ssd->channel_head[channel].chip_head[chip].token;
			plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;     //先处理所有的plane，这么做为了保证mutli plane的实用性

			if (plane == (ssd->parameter->plane_die - 1))
			{
				ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;											   //plane处理完成后，处理channel
				
				if (ssd->token == 0)																					   //1-0，所有channel处理完成，下个分配需要改die																				
				{
					//channel = ssd->token;
					//chip = ssd->channel_head[channel].token;
					ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = (die + 1) % ssd->parameter->die_chip;  //更改die
				}
				else																									 //0--1，channel未处理完成，继续处理改channel，此时不改变die
				{
					ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = die;      //不更改die
				}	
			}
		}
	}

	/******************************************************************************
	*根据上述分配方法找到channel，chip，die，plane后，再在这个里面找到active_block
	*接着获得ppn
	******************************************************************************/

	
	if (find_active_block(ssd, channel, chip, die, plane) == FAILURE)
	{
		//发现问题，可能预处理在同一个plane中不同的写块
		printf("the read operation is expand the capacity of SSD");
		getchar();
		return 0;
	}
	active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].active_block;
	if (write_page(ssd, channel, chip, die, plane, active_block, &ppn) == ERROR)
	{
		return 0;
	}
	

	return ppn;
}


/***************************************************************************************************
*函数功能是在所给的channel，chip，die，plane里面找到一个active_block然后再在这个block里面找到一个页，
*再利用find_ppn找到ppn。
****************************************************************************************************/
struct ssd_info *get_ppn(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, struct sub_request *sub)
{
	int old_ppn = -1;
	unsigned int ppn, lpn, full_page;
	unsigned int active_block;
	unsigned int block;
	unsigned int page, flag = 0, flag1 = 0;
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
	*利用函数find_active_block在channel，chip，die，plane找到活跃block
	*并且修改这个channel，chip，die，plane，active_block下的last_write_page和free_page_num
	**************************************************************************************/
	if (find_active_block(ssd, channel, chip, die, plane) == FAILURE)
	{
		printf("ERROR :there is no free page in channel:%d, chip:%d, die:%d, plane:%d\n", channel, chip, die, plane);
		return ssd;
	}

	active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].active_block;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page++;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].free_page_num--;

	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page>63)
	{
		printf("error! the last write page larger than 64!!\n");
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
	else                                                                            /*这个逻辑页进行了更新，需要将原来的页置为失效*/
	{
		ppn = ssd->dram->map->map_entry[lpn].pn;
		location = find_location(ssd, ppn);
		if (ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].lpn != lpn)
		{

			printf("\nError in get_ppn()\n");
			//getchar();
		}

		ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].valid_state = 0;             /*表示某一页失效，同时标记valid和free状态都为0*/
		ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].free_state = 0;              /*表示某一页失效，同时标记valid和free状态都为0*/
		ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].lpn = 0;
		ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].invalid_page_num++;

		/*******************************************************************************************
		*该block中全是invalid的页，可以直接删除，就在创建一个可擦除的节点，挂在location下的plane下面
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


	sub->ppn = ssd->dram->map->map_entry[lpn].pn;                                      /*修改sub子请求的ppn，location等变量*/
	sub->location->channel = channel;
	sub->location->chip = chip;
	sub->location->die = die;
	sub->location->plane = plane;
	sub->location->block = active_block;
	sub->location->page = page;

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_write_count++;
	ssd->program_count++;                                                           /*修改ssd的program_count,free_page等变量*/
	//ssd->channel_head[channel].program_count++;
	//ssd->channel_head[channel].chip_head[chip].program_count++;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page--;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].lpn = lpn;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].valid_state = sub->state;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].free_state = ((~(sub->state))&full_page);
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].written_count++;
	ssd->write_flash_count++;

	if (ssd->parameter->active_write == 0)                                            /*如果没有主动策略，只采用gc_hard_threshold，并且无法中断GC过程*/
	{                                                                               /*如果plane中的free_page的数目少于gc_hard_threshold所设定的阈值就产生gc操作*/
		if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page<(ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->gc_hard_threshold))
		{
			gc_node = (struct gc_operation *)malloc(sizeof(struct gc_operation));
			alloc_assert(gc_node, "gc_node");
			memset(gc_node, 0, sizeof(struct gc_operation));

			gc_node->next_node = NULL;
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
*这个函数的功能是根据参数channel，chip，die，plane，block，page，找到该物理页号
*函数的返回值就是这个物理页号
******************************************************************************/
unsigned int find_ppn(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, unsigned int block, unsigned int page)
{
	unsigned int ppn = 0;
	unsigned int i = 0;
	int page_plane = 0, page_die = 0, page_chip = 0;
	int page_channel[100];                  /*这个数组存放的是每个channel的page数目*/

#ifdef DEBUG
	printf("enter find_psn,channel:%d, chip:%d, die:%d, plane:%d, block:%d, page:%d\n", channel, chip, die, plane, block, page);
#endif

	/*********************************************
	*计算出plane，die，chip，channel中的page的数目
	**********************************************/
	page_plane = ssd->parameter->page_block*ssd->parameter->block_plane;
	page_die = page_plane*ssd->parameter->plane_die;
	page_chip = page_die*ssd->parameter->die_chip;
	while (i<ssd->parameter->channel_number)
	{
		page_channel[i] = ssd->parameter->chip_channel[i] * page_chip;
		i++;
	}

	/****************************************************************************
	*计算物理页号ppn，ppn是channel，chip，die，plane，block，page中page个数的总和
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
*函数的功能是根据物理页号ppn查找该物理页所在的channel，chip，die，plane，block，page
*得到的channel，chip，die，plane，block，page放在结构location中并作为返回值
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

	/*******************************************************************************
	*page_channel是一个channel中page的数目， ppn/page_channel就得到了在哪个channel中
	*用同样的办法可以得到chip，die，plane，block，page
	********************************************************************************/
	location->channel = ppn / page_channel;
	location->chip = (ppn%page_channel) / page_chip;
	location->die = ((ppn%page_channel) % page_chip) / page_die;
	location->plane = (((ppn%page_channel) % page_chip) % page_die) / page_plane;
	location->block = ((((ppn%page_channel) % page_chip) % page_die) % page_plane) / ssd->parameter->page_block;
	location->page = (((((ppn%page_channel) % page_chip) % page_die) % page_plane) % ssd->parameter->page_block) % ssd->parameter->page_block;

	return location;
}

/****************************************
*执行写子请求时，为普通的写子请求获取ppn
*****************************************/
Status get_ppn_for_normal_command(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request * sub)
{
	unsigned int die = 0;
	unsigned int plane = 0;
	if (sub == NULL)
	{
		return ERROR;
	}

	if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION)
	{
		die = ssd->channel_head[channel].chip_head[chip].token;
		plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
		get_ppn(ssd, channel, chip, die, plane, sub);

		if (ssd->parameter->dynamic_allocation_priority == 1)				//动态分配的优先级
		{
			//更新完die plane的令牌值
			if (plane == (ssd->parameter->plane_die - 1))
			{
				plane_cmplt = 1;
				//ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
			}
			else
			{
				plane_cmplt = 0;
				//ssd->channel_head[channel].chip_head[chip].token = die;
			}
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
		}
		else
		{
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;
			ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
		}
		compute_serve_time(ssd, channel, chip, die, &sub, 1, NORMAL);
 		return SUCCESS;
	}
}

/************************************************************************************************
*为高级命令获取ppn
*根据不同的命令，遵从在同一个block中顺序写的要求，选取可以进行写操作的ppn，跳过的ppn全部置为失效。
*在使用two plane操作时，为了寻找相同水平位置的页，可能需要直接找到两个完全空白的块，这个时候原来
*的块没有用完，只能放在这，等待下次使用，同时修改查找空白page的方法，将以前首先寻找free块改为，只
*要invalid block!=64即可。
*except find aim page, we should modify token and decide gc operation
*************************************************************************************************/
Status get_ppn_for_advanced_commands(struct ssd_info *ssd, unsigned int channel, unsigned int chip, struct sub_request * * subs, unsigned int subs_count, unsigned int command)
{
	unsigned int die = 0, plane = 0;
	unsigned int die_token = 0, plane_token = 0;
	struct sub_request * sub = NULL;
	unsigned int i = 0, j = 0, k = 0;
	unsigned int unvalid_subs_count = 0;
	unsigned int valid_subs_count = 0;
	unsigned int interleave_flag = FALSE;
	unsigned int multi_plane_falg = FALSE;
	unsigned int max_subs_num = 0;
	struct sub_request * first_sub_in_chip = NULL;
	struct sub_request * first_sub_in_die = NULL;
	struct sub_request * second_sub_in_die = NULL;
	unsigned int state = SUCCESS;
	unsigned int multi_plane_flag = FALSE;

	max_subs_num = ssd->parameter->die_chip*ssd->parameter->plane_die;

	if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION)                         /*动态分配操作*/
	{
		if (command == TWO_PLANE)
		{
			if (subs_count<2)
			{
				return ERROR;
			}
			die = ssd->channel_head[channel].chip_head[chip].token;
			for (j = 0; j<subs_count; j++)
			{
				if (j == 1)
				{
					state = find_level_page(ssd, channel, chip, die, subs[0], subs[1]);        /*寻找与subs[0]的ppn位置相同的subs[1]，执行TWO_PLANE高级命令*/
					if (state != SUCCESS)
					{
						get_ppn_for_normal_command(ssd, channel, chip, subs[0]);           /*没找到，那么就当普通命令来处理*/
						printf("lz:normal_wr_2\n");
						return FAILURE;
					}
					else
					{
						plane_cmplt = 1;
						valid_subs_count = 2;
					}
				}
				else if (j>1)		//超过三个的请求
				{
					state = make_level_page(ssd, subs[0], subs[j]);                         /*寻找与subs[0]的ppn位置相同的subs[j]，执行TWO_PLANE高级命令*/
					if (state != SUCCESS)
					{
						for (k = j; k<subs_count; k++)
						{
							subs[k] = NULL;
						}
						subs_count = j;
						break;
					}
					else
					{
						valid_subs_count++;
					}
				}
			}//for(j=0;j<subs_count;j++)
			ssd->m_plane_prog_count++;
			if (ssd->parameter->dynamic_allocation == 0)
			{
				ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
			}
			compute_serve_time(ssd, channel, chip, die, subs, valid_subs_count, TWO_PLANE);
			printf("lz:mutli_plane_wr_3\n");
			return SUCCESS;
		}//else if(command==TWO_PLANE)
		else
		{
			return ERROR;
		}
	}//if (ssd->parameter->allocation_scheme==DYNAMIC_ALLOCATION)
}



/******************************************************************************************下面是ftl层gc操作******************************************************************************************/
/************************************************************************************************************
*flag用来标记gc函数是在ssd整个都是idle的情况下被调用的（1），还是确定了channel，chip，die，plane被调用（0）
*进入gc函数，需要判断是否是不可中断的gc操作，如果是，需要将一整块目标block完全擦除后才算完成；如果是可中断的，
*在进行GC操作前，需要判断该channel，die是否有子请求在等待操作，如果没有则开始一步一步的操作，找到目标
*块后，一次执行一个copyback操作，跳出gc函数，待时间向前推进后，再做下一个copyback或者erase操作
*进入gc函数不一定需要进行gc操作，需要进行一定的判断，当处于硬阈值以下时，必须进行gc操作；当处于软阈值以下时，
*需要判断，看这个channel上是否有子请求在等待(有写子请求等待就不行，gc的目标die处于busy状态也不行)，如果
*有就不执行gc，跳出，否则可以执行一步操作
************************************************************************************************************/
unsigned int gc(struct ssd_info *ssd, unsigned int channel, unsigned int flag)
{
	unsigned int i;
	int flag_direct_erase = 1, flag_gc = 1, flag_invoke_gc = 1;
	unsigned int flag_priority = 0;
	struct gc_operation *gc_node = NULL, *gc_p = NULL;

	//主动gc
	if (flag == 1)                                                                       /*整个ssd都是IDEL的情况*/
	{
		for (i = 0; i<ssd->parameter->channel_number; i++)
		{
			flag_priority = 0;
			flag_direct_erase = 1;
			flag_gc = 1;
			flag_invoke_gc = 1;
			gc_node = NULL;
			gc_p = NULL;
			if ((ssd->channel_head[i].current_state == CHANNEL_IDLE) || (ssd->channel_head[i].next_state == CHANNEL_IDLE&&ssd->channel_head[i].next_state_predict_time <= ssd->current_time))
			{
				channel = i;
				if (ssd->channel_head[channel].gc_command != NULL)
				{
					gc_for_channel(ssd, channel);
				}
			}
		}
		return SUCCESS;

	}
	//被动gc
	else                                                                               /*只需针对某个特定的channel，chip，die进行gc请求的操作(只需对目标die进行判定，看是不是idle）*/
	{
		//只考虑全局分配，故静态分配直接去除
		if ((ssd->parameter->allocation_scheme == 0) && (ssd->parameter->dynamic_allocation == 0))
			//if ((ssd->parameter->allocation_scheme==1)||((ssd->parameter->allocation_scheme==0)&&(ssd->parameter->dynamic_allocation==1)))
		{
			if ((ssd->channel_head[channel].subs_r_head != NULL) || (ssd->channel_head[channel].subs_w_head != NULL))    /*队列上有请求，先服务请求*/
			{
				return 0;
			}
		}
		gc_for_channel(ssd, channel);
		return SUCCESS;
	}
}


/***************************************
*这个函数的功能是处理channel的每个gc操作
****************************************/
Status gc_for_channel(struct ssd_info *ssd, unsigned int channel)
{
	int flag_direct_erase = 1, flag_gc = 1, flag_invoke_gc = 1;
	unsigned int chip, die, plane, flag_priority = 0;
	unsigned int current_state = 0, next_state = 0;
	long long next_state_predict_time = 0;
	struct gc_operation *gc_node = NULL, *gc_p = NULL;

	/*******************************************************************************************
	*查找每一个gc_node，获取gc_node所在的chip的当前状态，下个状态，下个状态的预计时间
	*如果当前状态是空闲，或是下个状态是空闲而下个状态的预计时间小于当前时间，并且是不可中断的gc
	*那么就flag_priority令为1，否则为0
	********************************************************************************************/
	gc_node = ssd->channel_head[channel].gc_command;
	while (gc_node != NULL)
	{
		current_state = ssd->channel_head[channel].chip_head[gc_node->chip].current_state;
		next_state = ssd->channel_head[channel].chip_head[gc_node->chip].next_state;
		next_state_predict_time = ssd->channel_head[channel].chip_head[gc_node->chip].next_state_predict_time;
		if ((current_state == CHIP_IDLE) || ((next_state == CHIP_IDLE) && (next_state_predict_time <= ssd->current_time)))
		{
			if (gc_node->priority == GC_UNINTERRUPT)                                     /*这个gc请求是不可中断的，优先服务这个gc操作*/
			{
				flag_priority = 1;
				//printf("lz:gc_uninterrupt\n");
				break;
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

	if (gc_node->priority == GC_UNINTERRUPT)
	{
		flag_direct_erase = gc_direct_erase(ssd, channel, chip, die, plane);
		if (flag_direct_erase != SUCCESS)
		{
			flag_gc = uninterrupt_gc(ssd, channel, chip, die, plane);                         /*当一个完整的gc操作完成时（已经擦除一个块，回收了一定数量的flash空间），返回1，将channel上相应的gc操作请求节点删除*/
			if (flag_gc == 1)
			{
				delete_gc_node(ssd, channel, gc_node);
			}
		}
		else
		{
			delete_gc_node(ssd, channel, gc_node);
		}
		return SUCCESS;
	}
}



/*******************************************************************************************************************
*GC操作由某个plane的free块少于阈值进行触发，当某个plane被触发时，GC操作占据这个plane所在的die，因为die是一个独立单元。
*对一个die的GC操作，尽量做到四个plane同时erase，利用interleave erase操作。GC操作应该做到可以随时停止（移动数据和擦除
*时不行，但是间隙时间可以停止GC操作），以服务新到达的请求，当请求服务完后，利用请求间隙时间，继续GC操作。可以设置两个
*GC阈值，一个软阈值，一个硬阈值。软阈值表示到达该阈值后，可以开始主动的GC操作，利用间歇时间，GC可以被新到的请求中断；
*当到达硬阈值后，强制性执行GC操作，且此GC操作不能被中断，直到回到硬阈值以上。
*在这个函数里面，找出这个die所有的plane中，有没有可以直接删除的block，要是有的话，利用interleave two plane命令，删除
*这些block，否则有多少plane有这种直接删除的block就同时删除，不行的话，最差就是单独这个plane进行删除，连这也不满足的话，
*直接跳出，到gc_parallelism函数进行进一步GC操作。该函数寻找全部为invalid的块，直接删除，找到可直接删除的返回1，没有找
*到返回-1。
*********************************************************************************************************************/
int gc_direct_erase(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane)
{
	unsigned int lv_die = 0, lv_plane = 0;
	unsigned int interleaver_flag = FALSE, muilt_plane_flag = FALSE;
	unsigned int normal_erase_flag = TRUE;

	struct direct_erase * direct_erase_node1 = NULL;
	struct direct_erase * direct_erase_node2 = NULL;

	direct_erase_node1 = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].erase_node;
	if (direct_erase_node1 == NULL)
	{
		return FAILURE;
	}

	/********************************************************************************************************
	*当能处理TWOPLANE高级命令时，就在相应的channel，chip，die中两个不同的plane找到可以执行TWOPLANE操作的block
	*并置muilt_plane_flag为TRUE
	*********************************************************************************************************/
	if ((normal_erase_flag == TRUE))                              /*不是每个plane都有可以直接删除的block，只对当前plane进行普通的erase操作，或者只能执行普通命令*/
	{
		if (erase_planes(ssd, channel, chip, die, plane, NORMAL) == SUCCESS)
		{
			return SUCCESS;
		}
		else
		{
			return FAILURE;                                     /*目标的plane没有可以直接删除的block，需要寻找目标擦除块后在实施擦除操作*/
		}
	}
	return SUCCESS;
}


/*******************************************************************************************************************************************
*目标的plane没有可以直接删除的block，需要寻找目标擦除块后在实施擦除操作，用在不能中断的gc操作中，成功删除一个块，返回1，没有删除一个块返回-1
*在这个函数中，不用考虑目标channel,die是否是空闲的,擦除invalid_page_num最多的block
********************************************************************************************************************************************/
int uninterrupt_gc(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane)
{
	unsigned int i = 0, invalid_page = 0;
	unsigned int block, active_block, transfer_size, free_page, page_move_count = 0;                           /*记录失效页最多的块号*/
	struct local *  location = NULL;
	unsigned int total_invalid_page_num = 0;

	if (find_active_block(ssd, channel, chip, die, plane) != SUCCESS)                                           /*获取活跃块*/
	{
		printf("\n\n Error in uninterrupt_gc().\n");
		return ERROR;
	}

	active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].active_block;

	invalid_page = 0;
	transfer_size = 0;
	block = -1;
	for (i = 0; i<ssd->parameter->block_plane; i++)                                                           /*查找最多invalid_page的块号，以及最大的invalid_page_num*/
	{
		total_invalid_page_num += ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[i].invalid_page_num;
		if ((active_block != i) && (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[i].invalid_page_num>invalid_page))
		{
			invalid_page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[i].invalid_page_num;
			block = i;
		}
	}
	if (block == -1)
	{
		return 1;
	}

	//if(invalid_page<5)
	//{
	//printf("\ntoo less invalid page. \t %d\t %d\t%d\t%d\t%d\t%d\t\n",invalid_page,channel,chip,die,plane,block);
	//}

	free_page = 0;
	for (i = 0; i<ssd->parameter->page_block; i++)		                                                     /*逐个检查每个page，如果有有效数据的page需要移动到其他地方存储*/
	{
		if ((ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[i].free_state&PG_SUB) == 0x0000000f)
		{
			free_page++;
		}
		if (free_page != 0)
		{
			printf("\ntoo much free page. \t %d\t .%d\t%d\t%d\t%d\t\n", free_page, channel, chip, die, plane);
		}
		if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[i].valid_state>0) /*该页是有效页，需要copyback操作*/
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
			move_page(ssd, location, &transfer_size);                                                   /*真实的move_page操作*/
			page_move_count++;

			free(location);
			location = NULL;
		}
	}
	erase_operation(ssd, channel, chip, die, plane, block);	                                              /*执行完move_page操作后，就立即执行block的擦除操作*/

	ssd->channel_head[channel].current_state = CHANNEL_GC;
	ssd->channel_head[channel].current_time = ssd->current_time;
	ssd->channel_head[channel].next_state = CHANNEL_IDLE;
	ssd->channel_head[channel].chip_head[chip].current_state = CHIP_ERASE_BUSY;
	ssd->channel_head[channel].chip_head[chip].current_time = ssd->current_time;
	ssd->channel_head[channel].chip_head[chip].next_state = CHIP_IDLE;

	/***************************************************************
	*在可执行COPYBACK高级命令与不可执行COPYBACK高级命令这两种情况下，
	*channel下个状态时间的计算，以及chip下个状态时间的计算
	***************************************************************/
	ssd->channel_head[channel].next_state_predict_time = ssd->current_time + page_move_count*(7 * ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tR + 7 * ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tPROG) + transfer_size*SECTOR*(ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tRC);
	ssd->channel_head[channel].chip_head[chip].next_state_predict_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tBERS;

	return 1;
}

/*****************************************************************************************
*这个函数功能是为gc操作寻找新的ppn，因为在gc操作中需要找到新的物理块存放原来物理块上的数据
*在gc中寻找新物理块的函数，不会引起循环的gc操作
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

	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page>63)
	{
		printf("error! the last write page larger than 64!!\n");
		while (1){}
	}

	block = active_block;
	page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].last_write_page;

	ppn = find_ppn(ssd, channel, chip, die, plane, block, page);

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_write_count++;
	ssd->program_count++;
	//ssd->channel_head[channel].program_count++;
	//ssd->channel_head[channel].chip_head[chip].program_count++;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page--;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].written_count++;
	ssd->write_flash_count++;

	return ppn;

}
/*************************************************************
*函数的功能是当处理完一个gc操作时，需要把gc链上的gc_node删除掉
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
*这个函数的功能是处理INTERLEAVE_TWO_PLANE，INTERLEAVE，TWO_PLANE，NORMAL下的擦除的操作。
***************************************************************************************/
Status erase_planes(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die1, unsigned int plane1, unsigned int command)
{
	unsigned int die = 0;
	unsigned int plane = 0;
	unsigned int block = 0;
	struct direct_erase *direct_erase_node = NULL;
	unsigned int block0 = 0xffffffff;
	unsigned int block1 = 0;

	//不考虑高级命令
	if ((ssd->channel_head[channel].chip_head[chip].die_head[die1].plane_head[plane1].erase_node == NULL) || (command != NORMAL))
		//if((ssd->channel_head[channel].chip_head[chip].die_head[die1].plane_head[plane1].erase_node==NULL)||((command!=INTERLEAVE_TWO_PLANE)&&(command!=INTERLEAVE)&&(command!=TWO_PLANE)&&(command!=NORMAL)))     /*如果没有擦除操作，或者command不对，返回错误*/           
	{
		return ERROR;
	}

	/************************************************************************************************************
	*处理擦除操作时，首先要传送擦除命令，这是channel，chip处于传送命令的状态，即CHANNEL_TRANSFER，CHIP_ERASE_BUSY
	*下一状态是CHANNEL_IDLE，CHIP_IDLE。
	*************************************************************************************************************/
	block1 = ssd->channel_head[channel].chip_head[chip].die_head[die1].plane_head[plane1].erase_node->block;

	ssd->channel_head[channel].current_state = CHANNEL_TRANSFER;
	ssd->channel_head[channel].current_time = ssd->current_time;
	ssd->channel_head[channel].next_state = CHANNEL_IDLE;

	ssd->channel_head[channel].chip_head[chip].current_state = CHIP_ERASE_BUSY;
	ssd->channel_head[channel].chip_head[chip].current_time = ssd->current_time;
	ssd->channel_head[channel].chip_head[chip].next_state = CHIP_IDLE;

	if (command == NORMAL)																	/*普通命令NORMAL的处理*/
	{
		direct_erase_node = ssd->channel_head[channel].chip_head[chip].die_head[die1].plane_head[plane1].erase_node;
		block = direct_erase_node->block;
		ssd->channel_head[channel].chip_head[chip].die_head[die1].plane_head[plane1].erase_node = direct_erase_node->next_node;
		free(direct_erase_node);
		direct_erase_node = NULL;
		erase_operation(ssd, channel, chip, die1, plane1, block);

		ssd->direct_erase_count++;
		ssd->channel_head[channel].next_state_predict_time = ssd->current_time + 5 * ssd->parameter->time_characteristics.tWC;
		ssd->channel_head[channel].chip_head[chip].next_state_predict_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tWB + ssd->parameter->time_characteristics.tBERS;
	}
	else
	{
		return ERROR;
	}

	direct_erase_node = ssd->channel_head[channel].chip_head[chip].die_head[die1].plane_head[plane1].erase_node;

	if (((direct_erase_node) != NULL) && (direct_erase_node->block == block1))
	{
		return FAILURE;
	}
	else
	{
		return SUCCESS;
	}
}

/**************************************************************************************
*函数的功能是寻找活跃快，应为每个plane中都只有一个活跃块，只有这个活跃块中才能进行操作
***************************************************************************************/
Status  find_active_block(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane)
{
	unsigned int active_block = 0;
	unsigned int free_page_num = 0;
	unsigned int count = 0;
	//	int i, j, k, p, t;
	int lz = 0;

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


