/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName： ftl.c
Author: Zuo Lu 		Version: 1.1	Date:2017/05/12
Description: 
ftl layer: can not interrupt the global gc operation, gc operation to migrate valid pages using ordinary read and write operations, remove support copyback operation;

History:
<contributor>     <time>        <version>       <desc>									<e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim							617376665@qq.com
Zuo Lu			2017/05/12		  1.1			Support advanced commands:mutli plane   617376665@qq.com
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


/******************************************************************************************下面是ftl层map操作******************************************************************************************/

/*********************************************************************
*pre_process_page() handle all read request in advance and established 
*lpn<-->ppn of read request in advance ,in order to pre-processing trace 
*to prevent the read request is not read the data;
**********************************************************************/
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
	__int64 time;
	errno_t err;

	printf("\n");
	printf("begin pre_process_page.................\n");

	if ((err = fopen_s(&(ssd->tracefile), ssd->tracefilename, "r")) != 0)  
	{
		printf("the trace file can't open\n");
		return NULL;
	}

	full_page = ~(0xffffffff << (ssd->parameter->subpage_page));
	/*Calculate the maximum logical sector number for this ssd*/
	largest_lsn = (unsigned int)((ssd->parameter->chip_num*ssd->parameter->die_chip*ssd->parameter->plane_die*ssd->parameter->block_plane*ssd->parameter->page_block*ssd->parameter->subpage_page)*(1 - ssd->parameter->overprovide));

	while (fgets(buffer_request, 200, ssd->tracefile))
	{
		sscanf_s(buffer_request, "%I64u %d %d %d %d", &time, &device, &lsn, &size, &ope);
		fl++;
		trace_assert(time, device, lsn, size, ope);                       

		add_size = 0;                                                     /*Add_size is the size of the request that has been pre_processed*/

		if (ope == 1)                                                      
		{
			while (add_size<size)
			{
				lsn = lsn%largest_lsn;                                    /*To prevent access to lsn larger than the largest lsn*/
				sub_size = ssd->parameter->subpage_page - (lsn%ssd->parameter->subpage_page);
				if (add_size + sub_size >= size)                          /*This happens only if the size of a request is less than the size of a page or when the last page of a request is processed*/
				{
					sub_size = size - add_size;
					add_size += sub_size;
				}

				if ((sub_size>ssd->parameter->subpage_page) || (add_size>size))/*When preprocessing a sub-size, the size is greater than a page or has been processed size larger than size*/
				{
					printf("pre_process sub_size:%d\n", sub_size);
				}

				/*******************************************************************************************************
				*The logical page number lpn is calculated using the logical sector number lsn
				*To determine the dram in the mapping table map in the lpn position of the state
				*A，state=0，that has not been written before, write directly the size of the sub_size 
				*B，state>0，That has been written before, Merge sector of map state and request state
				********************************************************************************************************/
				lpn = lsn / ssd->parameter->subpage_page;
				if (ssd->dram->map->map_entry[lpn].state == 0)                
				{
					ppn = get_ppn_for_pre_process(ssd, lsn);
					location = find_location(ssd, ppn);
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
				else if (ssd->dram->map->map_entry[lpn].state>0)           
				{
					map_entry_new = set_entry_state(ssd, lsn, sub_size);      /*Get a new state, and with the original state phase to a state*/
					map_entry_old = ssd->dram->map->map_entry[lpn].state;
					modify = map_entry_new | map_entry_old;
					ppn = ssd->dram->map->map_entry[lpn].pn;				
					location = find_location(ssd, ppn);
					ssd->dram->map->map_entry[lsn / ssd->parameter->subpage_page].state = modify;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].valid_state = modify;
					ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_head[location->page].free_state = ((~modify)&full_page);

					free(location);
					location = NULL;
				}//else if(ssd->dram->map->map_entry[lpn].state>0)
				lsn = lsn + sub_size;                                       
				add_size += sub_size;                                      
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


/**************************************************
*The function is to get the state of a read request
***************************************************/
int set_entry_state(struct ssd_info *ssd, unsigned int lsn, unsigned int size)
{
	int temp, state, move;

	temp = ~(0xffffffff << size);
	move = lsn%ssd->parameter->subpage_page;
	state = temp << move;

	return state;
}

/**********************************************
*The function is to obtain the physical 
*page number ppn for the preprocessor function
**********************************************/
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

	if (ssd->parameter->allocation_scheme == 0)                           /*Dynamic way to get ppn*/
	{
		if (ssd->parameter->dynamic_allocation == 0)                      
		{
			if (ssd->parameter->dynamic_allocation_priority == 0)			//assign priority：channel>die>plane
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
			else																//assign priority：plane>channel>die
			{
				channel = ssd->token;
				chip = ssd->channel_head[channel].token;
				die = ssd->channel_head[channel].chip_head[chip].token;
				plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
				ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;     //Handle all planes, guarantee plane priority

				if (plane == (ssd->parameter->plane_die - 1))
				{
					ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;											   //Plane processing is completed, the processing channel, to ensure the priority of the channel
					if (ssd->token == 0)																					   																			
						ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = (die + 1) % ssd->parameter->die_chip;  //1-0,All channel processing is complete, the next allocation need to change die
					else																									 
						ssd->channel_head[ssd->token].chip_head[ssd->channel_head[channel].token].token = die;					//0--1，Channel untreated to complete, continue to change channel, this time does not change die
				}
			}
		}
	}

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
	*Use the function find_active_block() to find active blocks in channel, chip, die, plane
	*And modify the channel, chip, die, plane, active_block under the last_write_page and free_page_num
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

	if (ssd->parameter->active_write == 0)                                        
	{                                                                               /*If the number of free_page in plane is less than the threshold set by gc_hard_threshold, gc operation is generated*/
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

		if (ssd->parameter->dynamic_allocation_priority == 1)				
		{
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane + 1) % ssd->parameter->plane_die;			
			if (plane == (ssd->parameter->plane_die - 1))
				ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
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
	unsigned int die = 0, plane = 0;
	unsigned int die_token = 0, plane_token = 0;
	struct sub_request * sub = NULL;
	unsigned int i = 0, j = 0, k = 0;
	unsigned int unvalid_subs_count = 0;
	unsigned int valid_subs_count = 0;
	unsigned int max_subs_num = 0;
	unsigned int state = SUCCESS;

	max_subs_num = ssd->parameter->die_chip*ssd->parameter->plane_die;

	if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION)                        
	{
		if (command == TWO_PLANE)
		{
			if ( subs_count < ssd->parameter->plane_die )
			{
				return ERROR;
			}
			die = ssd->channel_head[channel].chip_head[chip].token;
			for (j = 0; j<subs_count; j++)
			{
				if (j == (ssd->parameter->plane_die-1) )
				{
					//state = find_level_page(ssd, channel, chip, die, subs[0], subs[1]);        /*Find subs[1] with the same page position as subs[0], execute TWO_PLANE advanced command*/
					state = find_level_page(ssd, channel, chip, die, subs, subs_count);
					if (state != SUCCESS)
					{
						get_ppn_for_normal_command(ssd, channel, chip, subs[0]);			   /*Ordinary command to deal with*/
						printf("lz:normal_wr_2\n");
						getchar();
						return FAILURE;
					}
					else
					{
						valid_subs_count = 2;
						ssd->channel_head[channel].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;    //Multi plane execution is successful, indicating that the current die execution is complete, update the token to the next
					}
				}
				else if (j> (ssd->parameter->plane_die-1) )									//beyong all plane sub_request
				{
					state = make_level_page(ssd, subs[0], subs[j]);                         /*Find subs[j] with the same ppn position as subs[0], execute TWO_PLANE advanced command*/
					printf("lz:normal_wr_2\n");
					getchar();
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
*Gc operation, for the invalid block, the use of mutli erase select two plane offset address of the same invalid block to erase,
*For the valid block, select the two planes within the invalid page of the largest block to erase, and migrate a valid page, 
*the purpose of this is to ensure that the use of mutli hit, that is, for the die, each erase the super block , In the mutli 
*plane write, only need to ensure that the page offset consistent, do not guarantee blcok offset address can be consistent.
************************************************************************************************************/
unsigned int gc(struct ssd_info *ssd, unsigned int channel, unsigned int flag)
{
	unsigned int i;
	int flag_direct_erase = 1, flag_gc = 1, flag_invoke_gc = 1;
	unsigned int flag_priority = 0;
	struct gc_operation *gc_node = NULL, *gc_p = NULL;

	printf("gc flag=%d\n",flag);
	//Active gc
	if (flag == 1)                                                                       /*The whole ssd is the case of IDEL*/
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
			else
			{
				return FAILURE;
			}
		}
		return SUCCESS;
	}
	//Passive gc
	else                                                                               /*Only for a specific channel, chip, die gc request operation (only the target die to determine whether to see is idle)*/
	{
		if ((ssd->parameter->allocation_scheme == 0) && (ssd->parameter->dynamic_allocation == 0))
		{
			if ((ssd->channel_head[channel].subs_r_head != NULL) || (ssd->channel_head[channel].subs_w_head != NULL))    /*Queue has a request, first service request*/
			{
				return 0;
			}
		}
		if (gc_for_channel(ssd, channel) == SUCCESS)
			return SUCCESS;
		else
			return FAILURE;
	}
}


/************************************************************
*this function is to handle every gc operation of the channel
************************************************************/
Status gc_for_channel(struct ssd_info *ssd, unsigned int channel)
{
	int flag_direct_erase = 1, flag_gc = 1, flag_invoke_gc = 1;
	unsigned int chip, die, plane, flag_priority = 0;
	unsigned int current_state = 0, next_state = 0;
	long long next_state_predict_time = 0;
	struct gc_operation *gc_node = NULL, *gc_p = NULL;
	unsigned int planeA, planeB;

	/*******************************************************************************************
	*Find each gc_node, get the current state of the chip where gc_node is located, the next state,
	*the expected time of the next state .If the current state is idle, or the next state is idle 
	*and the next state is expected to be less than the current time, and is not interrupted gc
	*Then the flag_priority order is 1, otherwise 0.
	********************************************************************************************/

	gc_node = ssd->channel_head[channel].gc_command;
	while (gc_node != NULL)
	{
		current_state = ssd->channel_head[channel].chip_head[gc_node->chip].current_state;
		next_state = ssd->channel_head[channel].chip_head[gc_node->chip].next_state;
		next_state_predict_time = ssd->channel_head[channel].chip_head[gc_node->chip].next_state_predict_time;
		if ((current_state == CHIP_IDLE) || ((next_state == CHIP_IDLE) && (next_state_predict_time <= ssd->current_time)))
		{
			if (gc_node->priority == GC_UNINTERRUPT)                                     /*this gc request is not interrupted, the priority service gc operation*/
			{
				flag_priority = 1;
				break;																	/*Processing the nearest free node on the current channel gc request chain*/
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

	if (gc_node->priority == GC_UNINTERRUPT)
	{
		flag_direct_erase = gc_direct_erase(ssd, channel, chip, die);
		if (flag_direct_erase != SUCCESS)
		{       
			flag_gc = uninterrupt_gc(ssd, channel, chip, die);							 /*When a complete gc operation is completed, return 1, the corresponding channel gc operation request node to delete*/
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
*GC operation in a number of plane selected two offset address of the same block to erase, and in the invalid block 
*on the table where the invalid block node, erase success, calculate the mutli plane erase operation of the implementation 
*time, channel chip status Change time
*********************************************************************************************************************/
int gc_direct_erase(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die)
{
	unsigned int i, erase_block = 0;			
	struct direct_erase * direct_erase_node1 = NULL;
	struct direct_erase * direct_erase_node2 = NULL;
	struct direct_erase * pre_erase_node2 = NULL;
	struct direct_erase * direct_erase_node = NULL;

	for ( i = 0; i < ssd->parameter->plane_die; i++)
	{
		direct_erase_node = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].erase_node;
		if (direct_erase_node == NULL)
			return FAILURE;

		//Perform mutli plane erase operation,and delete gc_node
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].erase_node = direct_erase_node->next_node;
		erase_operation(ssd, channel, chip, die, i, direct_erase_node->block);
		free(direct_erase_node);
		ssd->direct_erase_count++;
		direct_erase_node = NULL;
	}
	/************************************************************************************************************
	*When processing the erase operation, first send the erase command, which is channel, chip in the state of sending 
	*the command,--- CHANNEL_TRANSFER, CHIP_ERASE_BUSY,the next state is CHANNEL_IDLE, CHIP_IDLE.
	*************************************************************************************************************/
	ssd->channel_head[channel].current_state = CHANNEL_TRANSFER;
	ssd->channel_head[channel].current_time = ssd->current_time;
	ssd->channel_head[channel].next_state = CHANNEL_IDLE;
	ssd->channel_head[channel].chip_head[chip].current_state = CHIP_ERASE_BUSY;
	ssd->channel_head[channel].chip_head[chip].current_time = ssd->current_time;
	ssd->channel_head[channel].chip_head[chip].next_state = CHIP_IDLE;

	//erase the statistical time
	ssd->mplane_erase_conut++;
	ssd->channel_head[channel].next_state_predict_time = ssd->current_time + 14 * ssd->parameter->time_characteristics.tWC;
	ssd->channel_head[channel].chip_head[chip].next_state_predict_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tBERS;

	return SUCCESS;
}

/*******************************************************************************************************************************************
*The target plane can not be directly deleted by the block, need to find the target erase block after the implementation of the erase operation, 
*the successful deletion of a block, returns 1, does not delete a block returns -1
********************************************************************************************************************************************/
int uninterrupt_gc(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die)
{
	unsigned int i = 0, j = 0, p = 0 ,invalid_page = 0;
	unsigned int active_block1, active_block2, transfer_size, free_page, avg_page_move = 0;                           /*Record the maximum number of blocks that are invalid*/
	struct local *  location = NULL;
	unsigned int plane , move_plane;
	int block1, block2;
	
	unsigned int active_block;
	unsigned int block;
	unsigned int page_move_count = 0;
	struct direct_erase * direct_erase_node_tmp = NULL;
	struct direct_erase * pre_erase_node_tmp = NULL;
	unsigned int * erase_block;

	erase_block = (unsigned int*)malloc( ssd->parameter->plane_die * sizeof(erase_block));
	//gets active blocks within all plane
	for ( p = 0; p < ssd->parameter->plane_die; p++)
	{
		if ( find_active_block(ssd, channel, chip, die, p) != SUCCESS )
		{
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
			return 1;

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
		for (i = 0; i<ssd->parameter->page_block; i++)		                                                     /*Check each page one by one, if there is a valid data page need to move to other places to store*/
		{
			if ((ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[i].free_state&PG_SUB) == 0x0000000f)
			{
				free_page++;
			}
			if (free_page != 0)
			{
				printf("\ntoo much free page. \t %d\t .%d\t%d\t%d\t%d\t\n", free_page, channel, chip, die, plane); /*There are free pages, proved to be active blocks, blocks are not finished, can not be erased*/
				getchar();
			}

			if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[i].valid_state>0) /*The page is a valid page that requires a copyback operation*/
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
		erase_operation(ssd, channel, chip, die, plane, block);						/*After the completion of the move_page operation, the block erase operation is performed immediately*/
	}

	free(erase_block);
	erase_block = NULL;
	ssd->channel_head[channel].current_state = CHANNEL_GC;
	ssd->channel_head[channel].current_time = ssd->current_time;
	ssd->channel_head[channel].next_state = CHANNEL_IDLE;
	ssd->channel_head[channel].chip_head[chip].current_state = CHIP_ERASE_BUSY;
	ssd->channel_head[channel].chip_head[chip].current_time = ssd->current_time;
	ssd->channel_head[channel].chip_head[chip].next_state = CHIP_IDLE;

	/***************************************************************
	*In the case of executable COPYBACK advanced commands and non-executable 
	*COPYBACK advanced commands, the calculation of time updates and statistics
	***************************************************************/
	ssd->channel_head[channel].next_state_predict_time = ssd->current_time + page_move_count*(7 * ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tR + 7 * ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tPROG) + transfer_size*SECTOR*(ssd->parameter->time_characteristics.tWC + ssd->parameter->time_characteristics.tRC);
	ssd->channel_head[channel].chip_head[chip].next_state_predict_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tBERS;

	return 1;
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
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page--;
	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[active_block].page_head[page].written_count++;
	ssd->write_flash_count++;

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


