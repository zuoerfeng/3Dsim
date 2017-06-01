/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName： fcl.c
Author: Zuo Lu 		Version: 1.1	Date:2017/05/12
Description:
fcl layer: remove other high-level commands, leaving only mutli plane;

History:
<contributor>     <time>        <version>       <desc>									<e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim							617376665@qq.com
Zuo Lu			2017/05/12		  1.1			Support advanced commands:mutli plane   617376665@qq.com
*****************************************************************************************************************************/

#define _CRTDBG_MAP_ALLOC

#include <stdlib.h>
#include <crtdbg.h>

#include "flash.h"
#include "ssd.h"
#include "initialize.h"
#include "buffer.h"
#include "interface.h"
#include "ftl.h"
#include "fcl.h"

/**************************************************************************************
*Function function is given in the channel, chip, die above looking for reading requests
*The request for this child ppn corresponds to the ppn of the corresponding plane's register
*****************************************************************************************/
struct sub_request * find_read_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die)
{
	unsigned int plane = 0;
	int address_ppn = 0;
	struct sub_request *sub = NULL, *p = NULL;

	for (plane = 0; plane<ssd->parameter->plane_die; plane++)
	{
		address_ppn = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].add_reg_ppn;
		if (address_ppn != -1)
		{
			sub = ssd->channel_head[channel].subs_r_head;
			if (sub->ppn == address_ppn)
			{
				if (sub->next_node == NULL)
				{
					ssd->channel_head[channel].subs_r_head = NULL;
					ssd->channel_head[channel].subs_r_tail = NULL;
				}
				ssd->channel_head[channel].subs_r_head = sub->next_node;
			}
			while ((sub->ppn != address_ppn) && (sub->next_node != NULL))
			{
				if (sub->next_node->ppn == address_ppn)
				{
					p = sub->next_node;
					if (p->next_node == NULL)
					{
						sub->next_node = NULL;
						ssd->channel_head[channel].subs_r_tail = sub;
					}
					else
					{
						sub->next_node = p->next_node;
					}
					sub = p;
					break;
				}
				sub = sub->next_node;
			}
			if (sub->ppn == address_ppn)
			{
				sub->next_node = NULL;
				return sub;
			}
			else
			{
				printf("Error! Can't find the sub request.");
			}
		}
	}
	return NULL;
}

/*******************************************************************************
*function is to find a request to write, in two cases:
*1，If it is completely dynamic allocation on the ssd-> subs_w_head queue to find
*2，If not fully dynamic allocation on the ssd-> channel_head [channel] .subs_w_head queue to find
********************************************************************************/
struct sub_request * find_write_sub_request(struct ssd_info * ssd, unsigned int channel)
{
	struct sub_request * sub = NULL, *p = NULL;
	if ((ssd->parameter->allocation_scheme == 0) && (ssd->parameter->dynamic_allocation == 0))    
	{
		sub = ssd->subs_w_head;
		while (sub != NULL)
		{
			if (sub->current_state == SR_WAIT)
			{
				if (sub->update != NULL)                                                   
				{
					if ((sub->update->current_state == SR_COMPLETE) || ((sub->update->next_state == SR_COMPLETE) && (sub->update->next_state_predict_time <= ssd->current_time)))   //The updated page has been read
					{
						break;
					}
				}
				else
				{
					break;
				}
			}
			p = sub;
			sub = sub->next_node;
		}

		if (sub == NULL)      /*If you can not find a sub request that can be served, jump out of this for loop*/
		{
			return NULL;
		}

		if (sub != ssd->subs_w_head)
		{
			if (sub != ssd->subs_w_tail)
			{
				p->next_node = sub->next_node;
			}
			else
			{
				ssd->subs_w_tail = p;
				ssd->subs_w_tail->next_node = NULL;
			}
		}
		else
		{
			if (sub->next_node != NULL)
			{
				ssd->subs_w_head = sub->next_node;
			}
			else
			{
				ssd->subs_w_head = NULL;
				ssd->subs_w_tail = NULL;
			}
		}
		sub->next_node = NULL;
		if (ssd->channel_head[channel].subs_w_tail != NULL)
		{
			ssd->channel_head[channel].subs_w_tail->next_node = sub;
			ssd->channel_head[channel].subs_w_tail = sub;
		}
		else
		{
			ssd->channel_head[channel].subs_w_tail = sub;
			ssd->channel_head[channel].subs_w_head = sub;
		}
	}
	return sub;
}

/*********************************************************************************************
* function that specifically serves a read request
*1，Only when the current state of the sub request is SR_R_C_A_TRANSFER
*2，The current state of the read request is SR_COMPLETE or the next state is SR_COMPLETE and 
*the next state arrives less than the current time
**********************************************************************************************/
Status services_2_r_cmd_trans_and_complete(struct ssd_info * ssd)
{
	unsigned int i = 0;
	struct sub_request * sub = NULL, *p = NULL;
	

	for (i = 0; i<ssd->parameter->channel_number; i++)                                       /*This loop does not require the channel time, when the read request is completed, it will be removed from the channel queue*/
	{
		sub = ssd->channel_head[i].subs_r_head;

		while (sub != NULL)
		{
			if (sub->current_state == SR_R_C_A_TRANSFER)                                  /*Read command sent to the corresponding die set to busy, while modifying the state of sub request*/
			{
				if (sub->next_state_predict_time <= ssd->current_time)
				{
					go_one_step(ssd, sub, NULL, SR_R_READ, NORMAL);                      /*State transition processing function*/

				}
			}
			else if ((sub->current_state == SR_COMPLETE) || ((sub->next_state == SR_COMPLETE) && (sub->next_state_predict_time <= ssd->current_time)))
			{
				if (sub != ssd->channel_head[i].subs_r_head)                             /*if the request is completed, we delete it from read queue */
				{
					p->next_node = sub->next_node;
				}
				else
				{
					if (ssd->channel_head[i].subs_r_head != ssd->channel_head[i].subs_r_tail)
					{
						ssd->channel_head[i].subs_r_head = sub->next_node;
					}
					else
					{
						ssd->channel_head[i].subs_r_head = NULL;
						ssd->channel_head[i].subs_r_tail = NULL;
					}
				}
			}
			p = sub;
			sub = sub->next_node;
		}
	}

	return SUCCESS;
}

/**************************************************************************
*This function is also only deal with read requests, processing chip current state is CHIP_WAIT,
*Or the next state is CHIP_DATA_TRANSFER and the next state of the expected time is less than the current time of the chip
***************************************************************************/
Status services_2_r_data_trans(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag)
{
	int chip = 0;
	unsigned int die = 0, plane = 0, address_ppn = 0, die1 = 0;
	struct sub_request * sub = NULL, *p = NULL, *sub1 = NULL;
	struct sub_request * sub_twoplane_one = NULL, *sub_twoplane_two = NULL;
	struct sub_request * sub_interleave_one = NULL, *sub_interleave_two = NULL;
	for (chip = 0; chip<ssd->channel_head[channel].chip; chip++)
	{
		if ((ssd->channel_head[channel].chip_head[chip].current_state == CHIP_WAIT) || ((ssd->channel_head[channel].chip_head[chip].next_state == CHIP_DATA_TRANSFER) &&
			(ssd->channel_head[channel].chip_head[chip].next_state_predict_time <= ssd->current_time)))
		{
			for (die = 0; die<ssd->parameter->die_chip; die++)
			{
				sub = find_read_sub_request(ssd, channel, chip, die);                   /*In the channel, chip, die found in the request*/
				if (sub != NULL)
				{
					break;
				}
			}
			if (sub == NULL)
			{
				continue;
			} 
			if ((ssd->parameter->advanced_commands&AD_TWOPLANE_READ) == AD_TWOPLANE_READ)				
			{
				sub_twoplane_one = sub;
				sub_twoplane_two = NULL;
				/*To ensure that the sub_twoplane_two found is different from sub_twoplane_one, make add_reg_ppn = -1*/
				ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[sub->location->plane].add_reg_ppn = -1;
				sub_twoplane_two = find_read_sub_request(ssd, channel, chip, die);               /*In the same channel, chip, die looking for another read request*/

				/**************************************************************************************
				*If found, then the implementation of TWO_PLANE state transition function go_one_step
				*If not found so the implementation of the general order of the state transition function go_one_step
				***************************************************************************************/
				if (sub_twoplane_two == NULL)
				{
					go_one_step(ssd, sub_twoplane_one, NULL, SR_R_DATA_TRANSFER, NORMAL);
					*change_current_time_flag = 0;
					*channel_busy_flag = 1;

				}
				else
				{
					go_one_step(ssd, sub_twoplane_one, sub_twoplane_two, SR_R_DATA_TRANSFER, TWO_PLANE);
					*change_current_time_flag = 0;
					*channel_busy_flag = 1;

				}
			}
			else                                                                                 /*If ssd does not support advanced commands then execute an normal read request*/
			{
				printf("r_data_trans:normal command !\n");
				getchar();
				go_one_step(ssd, sub, NULL, SR_R_DATA_TRANSFER, NORMAL);
				*change_current_time_flag = 0;
				*channel_busy_flag = 1;
			}
			break;
		}

		if (*channel_busy_flag == 1)
		{
			break;
		}
	}
	return SUCCESS;
}


/*****************************************************************************************
*This function is also a service that only reads the child request, and is in a wait state
******************************************************************************************/
int services_2_r_wait(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag)
{
	unsigned int plane = 0, address_ppn = 0;
	struct sub_request * sub = NULL, *p = NULL;
	struct sub_request * sub_twoplane_one = NULL, *sub_twoplane_two = NULL;
	struct sub_request * sub_interleave_one = NULL, *sub_interleave_two = NULL;

	sub = ssd->channel_head[channel].subs_r_head;

	if ((ssd->parameter->advanced_commands&AD_TWOPLANE_READ) == AD_TWOPLANE_READ)         /*to find whether there are two sub request can be served by two plane operation*/
	{
		sub_twoplane_one = NULL;
		sub_twoplane_two = NULL;

		find_interleave_twoplane_sub_request(ssd, channel, &sub_twoplane_one, &sub_twoplane_two, TWO_PLANE);
		if (sub_twoplane_two != NULL)                                                     
		{
			go_one_step(ssd, sub_twoplane_one, sub_twoplane_two, SR_R_C_A_TRANSFER, TWO_PLANE);
			*change_current_time_flag = 0;
			*channel_busy_flag = 1;                                                       /*Has occupied the cycle of the bus, do not perform the die data back*/
		}
		else if ((ssd->parameter->advanced_commands&AD_INTERLEAVE) != AD_INTERLEAVE)      
		{
			while (sub != NULL)                                                            /*if there are read requests in queue, send one of them to target die*/
			{
				if (sub->current_state == SR_WAIT)
				{	   
					/*Note that the next judgment condition is different from the judgment condition in services_2_r_data_trans*/
					if ((ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].current_state == CHIP_IDLE) || ((ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].next_state == CHIP_IDLE) &&
						(ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].next_state_predict_time <= ssd->current_time)))
					{
						go_one_step(ssd, sub, NULL, SR_R_C_A_TRANSFER, NORMAL);
						*change_current_time_flag = 0;
						*channel_busy_flag = 1;                                           /*Has occupied the cycle of the bus, do not perform the die data back*/
						break;
					}
					else
					{
						*channel_busy_flag = 0;
					}
				}
				sub = sub->next_node;
			}
		}
	}

	/*******************************************************
	*Ssd can not perform an execution of an advanced command
	********************************************************/
	if (((ssd->parameter->advanced_commands&AD_INTERLEAVE) != AD_INTERLEAVE) && ((ssd->parameter->advanced_commands&AD_TWOPLANE_READ) != AD_TWOPLANE_READ))
	{
		printf("r_wait:normal command !\n");
		getchar();
		while (sub != NULL)                                                               /*if there are read requests in queue, send one of them to target chip*/
		{
			if (sub->current_state == SR_WAIT)
			{
				if ((ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].current_state == CHIP_IDLE) || ((ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].next_state == CHIP_IDLE) &&
					(ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].next_state_predict_time <= ssd->current_time)))
				{
					go_one_step(ssd, sub, NULL, SR_R_C_A_TRANSFER, NORMAL);
					*change_current_time_flag = 0;
					*channel_busy_flag = 1;                                             
					break;
				}
				else
				{
					/*because the die caused by the obstruction*/
				}
			}
			sub = sub->next_node;
		}
	}
	return SUCCESS;
}

/****************************************
Write the request function of the request
*****************************************/
Status services_2_write(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag)
{
	int j = 0, chip = 0;
	unsigned int k = 0;
	unsigned int  old_ppn = 0, new_ppn = 0;
	unsigned int chip_token = 0, die_token = 0;
	long long time = 0;
	struct sub_request * sub = NULL, *p = NULL;
	struct sub_request * sub_twoplane_one = NULL, *sub_twoplane_two = NULL;

	/************************************************************************************************************************
	*Because it is dynamic allocation, all write requests hanging in ssd-> subs_w_head, that is, do not know which allocation before writing on the channel
	*************************************************************************************************************************/
	if (ssd->subs_w_head != NULL)
	{
		if (ssd->parameter->allocation_scheme == 0)                                      
		{
			for (j = 0; j<ssd->channel_head[channel].chip; j++)							  //Traverse all the chips
			{
				if (ssd->subs_w_head == NULL)											  //The loop is stopped when the request is processed
				{
					break;
				}

				chip_token = ssd->channel_head[channel].token;                           
				if (*channel_busy_flag == 0)
				{
					if ((ssd->channel_head[channel].chip_head[chip_token].current_state == CHIP_IDLE) || ((ssd->channel_head[channel].chip_head[chip_token].next_state == CHIP_IDLE) && (ssd->channel_head[channel].chip_head[chip_token].next_state_predict_time <= ssd->current_time)))
					{
						if ((ssd->channel_head[channel].subs_w_head == NULL) && (ssd->subs_w_head == NULL))
						{
							break;
						}
						if (dynamic_advanced_process(ssd, channel, chip_token) == NULL)
						{
							*channel_busy_flag = 0;
							//did not successfully execute a write request, then the channel, chip die does not increase
						}
						else   
						{
							*channel_busy_flag = 1;                                
							ssd->channel_head[channel].token = (ssd->channel_head[channel].token + 1) % ssd->parameter->chip_channel[channel];	//Chip tokens increase, jump to the next chip, execute write request
							break;
						}
					}
					ssd->channel_head[channel].token = (ssd->channel_head[channel].token + 1) % ssd->parameter->chip_channel[channel];  //The current chip is busy and jumps to the next chip execution
				}
			}
		}
	}
	else
	{
		//printf("there is no write sub_request\n");
		//return FAILURE;
	}
	return SUCCESS;
}




/****************************************************************************************************************************
*When ssd supports advanced commands, the function of this function is to deal with high-level command write request
*According to the number of requests, decide which type of advanced command to choose (this function only deal with write requests, 
*read requests have been assigned to each channel, so the implementation of the election between the corresponding command)
*****************************************************************************************************************************/
struct ssd_info *dynamic_advanced_process(struct ssd_info *ssd, unsigned int channel, unsigned int chip)
{
	unsigned int subs_count = 0;
	unsigned int update_count = 0;
	unsigned int plane_count = 0;                                                                
	unsigned int plane_place;                                                             /*record which plane has sub request in static allocation*/
	struct sub_request *sub = NULL, *p = NULL, *sub0 = NULL, *sub1 = NULL, *sub2 = NULL, *sub3 = NULL, *sub0_rw = NULL, *sub1_rw = NULL, *sub2_rw = NULL, *sub3_rw = NULL;
	struct sub_request ** subs = NULL;
	unsigned int max_sub_num = 0;
	unsigned int die_token = 0, plane_token = 0;
	unsigned int * plane_bits = NULL;

	unsigned int mask = 0x00000001;
	unsigned int i = 0, j = 0;

	plane_count = ssd->parameter->plane_die;
	max_sub_num = (ssd->parameter->die_chip)*(ssd->parameter->plane_die);    //This takes into account the parallelism between chip die, so max_sub_num represents the maximum number of concurrent requests on a chip
	subs = (struct sub_request **)malloc(max_sub_num*sizeof(struct sub_request *));
	alloc_assert(subs, "sub_request");

	for (i = 0; i<max_sub_num; i++)
	{
		subs[i] = NULL;  //executable request array
	}
	update_count = 0;

	if ((ssd->parameter->allocation_scheme == 0))                                           /*Full dynamic allocation, you need to select the wait-to-service sub-request from ssd-> subs_w_head*/
	{
		if (ssd->parameter->dynamic_allocation == 0)
			sub = ssd->subs_w_head;
		else
			sub = ssd->channel_head[channel].subs_w_head;

		subs_count = 0;
		while ((sub != NULL) && (subs_count<max_sub_num))
		{
			if (sub->current_state == SR_WAIT)
			{
				if ((sub->update == NULL) || ((sub->update != NULL) && ((sub->update->current_state == SR_COMPLETE) || ((sub->update->next_state == SR_COMPLETE) && (sub->update->next_state_predict_time <= ssd->current_time)))))    //没有需要提前读出的页
				{
					subs[subs_count] = sub;			
					subs_count++;					
				}
			}

			if (sub->update_read_flag == 1)
				update_count++;

			p = sub;
			sub = sub->next_node;
		}

		if (update_count > ssd->update_sub_request)
			ssd->update_sub_request = update_count; 
		
		if (update_count > ssd->parameter->update_reqeust_max)
		{
			printf("update sub request is full!\n");
			ssd->buffer_full_flag = 1;  //blcok the buffer
		}
		else
			ssd->buffer_full_flag = 0;


		//Write more than two requests, that can be advanced orders
		if (subs_count >= ssd->parameter->plane_die)
		{	
			//Request a maximum of plane mutations of plane_die at a time
			if ((ssd->parameter->advanced_commands&AD_TWOPLANE) == AD_TWOPLANE)
			{
				if (subs_count>ssd->parameter->plane_die)
				{
					for (i = ssd->parameter->plane_die; i<subs_count; i++)
					{
						subs[i] = NULL;
					}
					subs_count = ssd->parameter->plane_die;
				}
				get_ppn_for_advanced_commands(ssd, channel, chip, subs, subs_count, TWO_PLANE);
			}
			else
			{
				for (i = 1; i<subs_count; i++)
				{
					subs[i] = NULL;
				}
				subs_count = 1;
				get_ppn_for_normal_command(ssd, channel, chip, subs[0]);
				printf("lz:normal_wr_1\n");
				getchar();
			}
		}//if(subs_count>=ssd->parameter->plane_die)
		else if (subs_count > 0)
		{
			//get_ppn_for_normal_command(ssd, channel, chip, subs[0]);
			while (subs_count < ssd->parameter->plane_die)
			{
				getout2buffer(ssd, NULL, subs[0]->total_request);
				//重新遍历整个写请求链，取出对应的两个写子请求
				for (i = 0; i<max_sub_num; i++)
				{
					subs[i] = NULL;
				}
				sub = ssd->subs_w_head;
				subs_count = 0;
				while ((sub != NULL) && (subs_count<max_sub_num))
				{
					if (sub->current_state == SR_WAIT)
					{
						if ((sub->update == NULL) || ((sub->update != NULL) && ((sub->update->current_state == SR_COMPLETE) || ((sub->update->next_state == SR_COMPLETE) && (sub->update->next_state_predict_time <= ssd->current_time)))))    //没有需要提前读出的页
						{
							subs[subs_count] = sub;
							subs_count++;
						}
					}
					p = sub;
					sub = sub->next_node;
				}
			}
			get_ppn_for_advanced_commands(ssd, channel, chip, subs, subs_count, TWO_PLANE);
		}
		else	//subs_count = 0																	
		{
			for (i = 0; i<max_sub_num; i++)
			{
				subs[i] = NULL;
			}
			free(subs);
			subs = NULL;
			free(plane_bits);
			return NULL;
		}
	} 

	for (i = 0; i<max_sub_num; i++)
	{
		subs[i] = NULL;
	}
	free(subs);
	subs = NULL;
	free(plane_bits);
	return ssd;
}



/*****************************************************************
*the function is to let sub0, sub1 ppn where the same page position
******************************************************************/
Status make_level_page(struct ssd_info * ssd, struct sub_request * sub0, struct sub_request * sub1)
{
	unsigned int i = 0, j = 0, k = 0;
	unsigned int channel = 0, chip = 0, die = 0, plane0 = 0, plane1 = 0, block0 = 0, block1 = 0, page0 = 0, page1 = 0;
	unsigned int active_block0 = 0, active_block1 = 0;
	unsigned int old_plane_token = 0;

	if ((sub0 == NULL) || (sub1 == NULL) || (sub0->location == NULL))
	{
		return ERROR;
	}
	channel = sub0->location->channel;
	chip = sub0->location->chip;
	die = sub0->location->die;
	plane0 = sub0->location->plane;
	block0 = sub0->location->block;
	page0 = sub0->location->page;
	old_plane_token = ssd->channel_head[channel].chip_head[chip].die_head[die].token;

	/***********************************************************************************************
	*Dynamic allocation of the case:
	*Sub1 plane is based on sub0 ssd-> channel_head [channel] .chip_head [chip] .die_head [die] .token token obtained
	*Sub1 channel, chip, die, block, page and sub0 are the same
	************************************************************************************************/
	if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION)
	{
		old_plane_token = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
		for (i = 0; i<ssd->parameter->plane_die; i++)
		{
			plane1 = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
			if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane1].add_reg_ppn == -1)
			{
				find_active_block(ssd, channel, chip, die, plane1);                               /*Locate active blocks in plane1*/
				block1 = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane1].active_block;
				if (block1 == block0)
				{
					page1 = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane1].blk_head[block1].last_write_page + 1;
					if (page1 == page0)
					{
						break;
					}
					else if (page1<page0)
					{
						if (ssd->parameter->greed_MPW_ad == 1)                                  /*Allow greedy use of advanced commands*/
						{
							//make_same_level(ssd,channel,chip,die,plane1,active_block1,page0); /*Small page address to the big page address by*/
							make_same_level(ssd, channel, chip, die, plane1, block1, page0);
							break;
						}
					}
				}//if(block1==block0)
			}
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane1 + 1) % ssd->parameter->plane_die;
		}//for(i=0;i<ssd->parameter->plane_die;i++)
		if (i<ssd->parameter->plane_die)
		{
			flash_page_state_modify(ssd, sub1, channel, chip, die, plane1, block1, page0);          /*The function of this function is to update the page page corresponding to the physical page and location also map table*/
			//flash_page_state_modify(ssd,sub1,channel,chip,die,plane1,block1,page1);
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (plane1 + 1) % ssd->parameter->plane_die;
			return SUCCESS;
		}
		else
		{
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane_token;
			return FAILURE;
		}
	}
}

/******************************************************************************************************
*The function of the function is to find two pages of the same horizontal position for the two plane 
*command, and modify the statistics, modify the status of the page
*******************************************************************************************************/
//Status find_level_page(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request *subA, struct sub_request *subB)
Status find_level_page(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **sub, unsigned int subs_count)
{
	unsigned int i, planeA, planeB, active_blockA, active_blockB, pageA, pageB, aim_page = 0, old_plane;
	struct gc_operation *gc_node;
	unsigned int gc_add;

	unsigned int plane, active_block, page,tmp_page,equal_flag;
	unsigned int *page_place;

	page_place = (unsigned int *)malloc(ssd->parameter->plane_die*sizeof(page_place));
	old_plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
	
	for (i = 0; i < ssd->parameter->plane_die; i++)
	{
		find_active_block(ssd, channel, chip, die, i);
		active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].active_block;
		page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].blk_head[active_block].last_write_page + 1;
		page_place[i] = page;
	}

	equal_flag = 1;
	for (i = 0; i < (ssd->parameter->plane_die - 1); i++)
	{
		if (page_place[i] != page_place[i + 1])
		{
			equal_flag = 0;
			break;
		}
	}
	
	//判断所有的page是否相等，如果相等，执行mutli plane，如果不相等，贪婪的使用，将所有的page向最大的page靠近
	if (equal_flag == 1)	//page偏移地址一致
	{
		for (i = 0; i < ssd->parameter->plane_die; i++)
		{
			active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].active_block;
			flash_page_state_modify(ssd, sub[i], channel, chip, die, i, active_block, page_place[i]);
		}
	}
	else				    //page偏移地址不一致
	{
		if (ssd->parameter->greed_MPW_ad == 1)                                             /*greedily use advanced commands*/
		{
			for (i = 0; i < ssd->parameter->plane_die ; i++)
			{
				if (page_place[i] > aim_page)
					aim_page = page_place[i];
			}

			for (i = 0; i < ssd->parameter->plane_die; i++)
			{
				active_block = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].active_block;
				if (page_place[i] != aim_page)
					make_same_level(ssd, channel, chip, die, i, active_block, aim_page);
				flash_page_state_modify(ssd, sub[i], channel, chip, die, i, active_block, aim_page);
			}
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane;
		}
		else                                                                             /*can not greedy the use of advanced orders*/
		{
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane;
			for (i = 0; i < subs_count; i++)
				sub[i] = NULL;
			free(page_place);
			return FAILURE;
		}
	}
	gc_add = 1;
	for ( i = 0; i < ssd->parameter->plane_die; i++)
	{
		if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[i].free_page >= (ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->gc_hard_threshold))
			gc_add = 0;
	}
	if (gc_add == 1)		//produce a gc reqeuest and add gc_node to the channel
	{
		gc_node = (struct gc_operation *)malloc(sizeof(struct gc_operation));
		alloc_assert(gc_node, "gc_node");
		memset(gc_node, 0, sizeof(struct gc_operation));

		gc_node->next_node = NULL;
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
	free(page_place);
	return SUCCESS;
}




/*************************************************************
*the function is to have two different page positions the same
**************************************************************/
struct ssd_info *make_same_level(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, unsigned int block, unsigned int aim_page)
{
	int i = 0, step, page;
	struct direct_erase *new_direct_erase, *direct_erase_node;

	page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].last_write_page + 1;                  /*The page number of the current block that needs to be adjusted*/
	step = aim_page - page;
	while (i<step)
	{
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[page + i].valid_state = 0;     
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[page + i].free_state = 0;     
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[page + i].lpn = 0;

		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].invalid_page_num++;
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].free_page_num--;
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page--;

		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_write_count++;
		i++;
	}

	ssd->waste_page_count += step;

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].last_write_page = aim_page - 1;

	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].invalid_page_num == ssd->parameter->page_block)    /*The block is invalid in the page, it can directly delete*/
	{
		new_direct_erase = (struct direct_erase *)malloc(sizeof(struct direct_erase));
		alloc_assert(new_direct_erase, "new_direct_erase");
		memset(new_direct_erase, 0, sizeof(struct direct_erase));

		direct_erase_node = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].erase_node;
		if (direct_erase_node == NULL)
		{
			ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].erase_node = new_direct_erase;
		}
		else
		{
			new_direct_erase->next_node = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].erase_node;
			ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].erase_node = new_direct_erase;
		}
	}

	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].last_write_page>63)
	{
		printf("error! the last write page larger than 64!!\n");
		while (1){}
	}

	return ssd;
}



/****************************************************************************
*this function is to calculate the processing time and the state transition 
*of the processing when processing the write request for the advanced command
*****************************************************************************/
struct ssd_info *compute_serve_time(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **subs, unsigned int subs_count, unsigned int command)
{
	unsigned int i = 0;
	unsigned int max_subs_num = 0;
	struct sub_request *sub = NULL, *p = NULL;
	struct sub_request * last_sub = NULL;
	max_subs_num = ssd->parameter->die_chip*ssd->parameter->plane_die;

	if (command == TWO_PLANE)
	{
		for (i = 0; i<max_subs_num; i++)
		{
			if (subs[i] != NULL)
			{

				subs[i]->current_state = SR_W_TRANSFER;
				if (last_sub == NULL)
				{
					subs[i]->current_time = ssd->current_time;
				}
				else
				{
					subs[i]->current_time = last_sub->complete_time + ssd->parameter->time_characteristics.tDBSY;
				}

				subs[i]->next_state = SR_COMPLETE;
				subs[i]->next_state_predict_time = subs[i]->current_time + 7 * ssd->parameter->time_characteristics.tWC + (subs[i]->size*ssd->parameter->subpage_capacity)*ssd->parameter->time_characteristics.tWC;
				subs[i]->complete_time = subs[i]->next_state_predict_time;
				last_sub = subs[i];

				delete_from_channel(ssd, channel, subs[i]);
			}
		}
		ssd->channel_head[channel].current_state = CHANNEL_TRANSFER;
		ssd->channel_head[channel].current_time = ssd->current_time;
		ssd->channel_head[channel].next_state = CHANNEL_IDLE;
		ssd->channel_head[channel].next_state_predict_time = last_sub->complete_time;

		ssd->channel_head[channel].chip_head[chip].current_state = CHIP_WRITE_BUSY;
		ssd->channel_head[channel].chip_head[chip].current_time = ssd->current_time;
		ssd->channel_head[channel].chip_head[chip].next_state = CHIP_IDLE;
		ssd->channel_head[channel].chip_head[chip].next_state_predict_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tPROG;
	}
	else if (command == NORMAL)
	{
		subs[0]->current_state = SR_W_TRANSFER;
		subs[0]->current_time = ssd->current_time;
		subs[0]->next_state = SR_COMPLETE;
		subs[0]->next_state_predict_time = ssd->current_time + 7 * ssd->parameter->time_characteristics.tWC + (subs[0]->size*ssd->parameter->subpage_capacity)*ssd->parameter->time_characteristics.tWC;
		subs[0]->complete_time = subs[0]->next_state_predict_time;

		delete_from_channel(ssd, channel, subs[0]);

		ssd->channel_head[channel].current_state = CHANNEL_TRANSFER;
		ssd->channel_head[channel].current_time = ssd->current_time;
		ssd->channel_head[channel].next_state = CHANNEL_IDLE;
		ssd->channel_head[channel].next_state_predict_time = subs[0]->complete_time;

		ssd->channel_head[channel].chip_head[chip].current_state = CHIP_WRITE_BUSY;
		ssd->channel_head[channel].chip_head[chip].current_time = ssd->current_time;
		ssd->channel_head[channel].chip_head[chip].next_state = CHIP_IDLE;
		ssd->channel_head[channel].chip_head[chip].next_state_predict_time = ssd->channel_head[channel].next_state_predict_time + ssd->parameter->time_characteristics.tPROG;
	}
	else
	{
		return NULL;
	}

	return ssd;

}

/*****************************************************************************************
*Function is to remove the request from ssd-> subs_w_head or ssd-> channel_head [channel] .subs_w_head
******************************************************************************************/
struct ssd_info *delete_from_channel(struct ssd_info *ssd, unsigned int channel, struct sub_request * sub_req)
{
	struct sub_request *sub, *p;
	if ((ssd->parameter->allocation_scheme == 0) && (ssd->parameter->dynamic_allocation == 0))
	{
		sub = ssd->subs_w_head;
	}
	else
	{
		sub = ssd->channel_head[channel].subs_w_head;
	}
	p = sub;

	while (sub != NULL)
	{
		if (sub == sub_req)
		{
			if ((ssd->parameter->allocation_scheme == 0) && (ssd->parameter->dynamic_allocation == 0))
			{
				if (ssd->parameter->ad_priority2 == 0)
				{
					ssd->real_time_subreq--;
				}

				if (sub == ssd->subs_w_head)                                                     /*This sub request is removed from the sub request queue*/
				{
					if (ssd->subs_w_head != ssd->subs_w_tail)
					{
						ssd->subs_w_head = sub->next_node;
						sub = ssd->subs_w_head;
						continue;
					}
					else
					{
						ssd->subs_w_head = NULL;
						ssd->subs_w_tail = NULL;
						p = NULL;
						break;
					}
				}//if (sub==ssd->subs_w_head) 
				else
				{
					if (sub->next_node != NULL)
					{
						p->next_node = sub->next_node;
						sub = p->next_node;
						continue;
					}
					else
					{
						ssd->subs_w_tail = p;
						ssd->subs_w_tail->next_node = NULL;
						break;
					}
				}
			}//if ((ssd->parameter->allocation_scheme==0)&&(ssd->parameter->dynamic_allocation==0)) 
		}//if (sub==sub_req)
		p = sub;
		sub = sub->next_node;
	}//while (sub!=NULL)

	return ssd;
}


/****************************************************************************************
*Function is to deal with read the request of the advanced order, you need to find one_page 
*match another page that is two_page
*****************************************************************************************/
struct sub_request *find_interleave_twoplane_page(struct ssd_info *ssd, struct sub_request *one_page, unsigned int command)
{
	struct sub_request *two_page;

	if (one_page->current_state != SR_WAIT)
	{
		return NULL;
	}
	if (((ssd->channel_head[one_page->location->channel].chip_head[one_page->location->chip].current_state == CHIP_IDLE) || ((ssd->channel_head[one_page->location->channel].chip_head[one_page->location->chip].next_state == CHIP_IDLE) &&
		(ssd->channel_head[one_page->location->channel].chip_head[one_page->location->chip].next_state_predict_time <= ssd->current_time))))
	{
		two_page = one_page->next_node;
		if (command == TWO_PLANE)
		{
			while (two_page != NULL)
			{
				if (two_page->current_state != SR_WAIT)
				{
					two_page = two_page->next_node;
				}
				else if ((one_page->location->chip == two_page->location->chip) && (one_page->location->die == two_page->location->die) && (one_page->location->block == two_page->location->block) && (one_page->location->page == two_page->location->page))
				{
					if (one_page->location->plane != two_page->location->plane)
					{
						return two_page;                                                       /*find a page with one_page can perform two plane operations*/
					}
					else
					{
						two_page = two_page->next_node;
					}
				}
				else
				{
					two_page = two_page->next_node;
				}
			}//while (two_page!=NULL)
			if (two_page == NULL)                                                               /*did not find a page that can perform two_plane operations with one_page, need to move one_page back one node*/
			{
				return NULL;
			}
		}//if(command==TWO_PLANE)		
	}
	else
	{
		return NULL;
	}
}


/*************************************************************************
*In dealing with the sub-request request advanced command, the use of this 
*or find the implementation of advanced orders sub_request
**************************************************************************/
int find_interleave_twoplane_sub_request(struct ssd_info * ssd, unsigned int channel, struct sub_request ** sub_request_one, struct sub_request ** sub_request_two, unsigned int command)
{
	*sub_request_one = ssd->channel_head[channel].subs_r_head;

	while ((*sub_request_one) != NULL)
	{
		(*sub_request_two) = find_interleave_twoplane_page(ssd, *sub_request_one, command);                //Find two read sub requests that can do either two_plane or interleave, including location conditions and time conditions

		if (*sub_request_two == NULL)
		{
			*sub_request_one = (*sub_request_one)->next_node;
		}
		else if (*sub_request_two != NULL)                                                            //Two pages that can perform two plane operations are found
		{
			break;
		}
	}

	if (*sub_request_two != NULL)
	{
		return SUCCESS;
	}
	else
	{
		return FAILURE;
	}

}


/***********************************************************************************************************
*1.The state transition of the child request, and the calculation of the time, are handled by this function
*2.The state of the execution of the normal command, and the calculation of the time, are handled by this function
****************************************************************************************************************/
Status go_one_step(struct ssd_info * ssd, struct sub_request * sub1, struct sub_request *sub2, unsigned int aim_state, unsigned int command)
{
	unsigned int i = 0, j = 0, k = 0, m = 0;
	long long time = 0;
	struct sub_request * sub = NULL;
	struct sub_request * sub_twoplane_one = NULL, *sub_twoplane_two = NULL;
	struct sub_request * sub_interleave_one = NULL, *sub_interleave_two = NULL;
	struct local * location = NULL;

	struct buffer_group *update_buffer_node = NULL;

	if (sub1 == NULL)
	{
		return ERROR;
	}

	/***************************************************************************************************
	*When dealing with ordinary commands, the target state of the read request is divided into the following
	*cases: SR_R_READ, SR_R_C_A_TRANSFER, SR_R_DATA_TRANSFER
	*
	*The target status of the write request is only SR_W_TRANSFER
	****************************************************************************************************/
	if (command == NORMAL)
	{
		sub = sub1;
		location = sub1->location;

		switch (aim_state)
		{
		case SR_R_READ:
		{
			/*****************************************************************************************************
			*This target state is flash in the state of reading data, sub the next state should be transmitted data SR_R_DATA_TRANSFER.
			*Then has nothing to do with the channel, only with the chip so to modify the chip status CHIP_READ_BUSY, the next state is CHIP_DATA_TRANSFER
			******************************************************************************************************/
			sub->current_time = ssd->current_time;
			sub->current_state = SR_R_READ;
			sub->next_state = SR_R_DATA_TRANSFER;
			sub->next_state_predict_time = ssd->current_time + ssd->parameter->time_characteristics.tR;

			ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_READ_BUSY;
			ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_DATA_TRANSFER;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = ssd->current_time + ssd->parameter->time_characteristics.tR;

			break;
		}
		case SR_R_C_A_TRANSFER:
		{
			/*******************************************************************************************************
			*When the target state is the command address transfer, the next state of sub is SR_R_READ
			*This state and channel, chip, so to modify the channel, chip status were CHANNEL_C_A_TRANSFER, CHIP_C_A_TRANSFER
			*The next status is CHANNEL_IDLE, CHIP_READ_BUSY
			*******************************************************************************************************/
			sub->current_time = ssd->current_time;
			sub->current_state = SR_R_C_A_TRANSFER;
			sub->next_state = SR_R_READ;
			sub->next_state_predict_time = ssd->current_time + 7 * ssd->parameter->time_characteristics.tWC;
			sub->begin_time = ssd->current_time;

			ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].add_reg_ppn = sub->ppn;
			//printf("r_data_trans read\n");
			ssd->read_count++;
			ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_read_count++;

			ssd->channel_head[location->channel].current_state = CHANNEL_C_A_TRANSFER;
			ssd->channel_head[location->channel].current_time = ssd->current_time;
			ssd->channel_head[location->channel].next_state = CHANNEL_IDLE;
			ssd->channel_head[location->channel].next_state_predict_time = ssd->current_time + 7 * ssd->parameter->time_characteristics.tWC;

			ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_C_A_TRANSFER;
			ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_READ_BUSY;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = ssd->current_time + 7 * ssd->parameter->time_characteristics.tWC;

			break;

		}
		case SR_R_DATA_TRANSFER:
		{
			/**************************************************************************************************************
			*When the target state is data transfer, the next state of sub is the completion state. SR_COMPLETE
			*The state of the deal with the channel, chip, so channel, chip current state into CHANNEL_DATA_TRANSFER, CHIP_DATA_TRANSFER
			*The next state is CHANNEL_IDLE, CHIP_IDLE.
			***************************************************************************************************************/
			sub->current_time = ssd->current_time;
			sub->current_state = SR_R_DATA_TRANSFER;
			sub->next_state = SR_COMPLETE;
			sub->next_state_predict_time = ssd->current_time + (sub->size*ssd->parameter->subpage_capacity)*ssd->parameter->time_characteristics.tRC;
			sub->complete_time = sub->next_state_predict_time;

			if (sub->update_read_flag == 1)
				sub->update_read_flag = 0;
			
			ssd->channel_head[location->channel].current_state = CHANNEL_DATA_TRANSFER;
			ssd->channel_head[location->channel].current_time = ssd->current_time;
			ssd->channel_head[location->channel].next_state = CHANNEL_IDLE;
			ssd->channel_head[location->channel].next_state_predict_time = sub->next_state_predict_time;

			ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_DATA_TRANSFER;
			ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_IDLE;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = sub->next_state_predict_time;

			ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].add_reg_ppn = -1;

			break;
		}
		case SR_W_TRANSFER:
		{
			/******************************************************************************************************
			*This is the time to deal with write requests, state changes, and time calculations
			*Write requests are from the top of the plane to transfer data, so that you can put a few states as a state 
			*to deal with, as SR_W_TRANSFER this state to deal with, sub next state is complete state
			*At this time channel, chip current state into CHANNEL_TRANSFER, CHIP_WRITE_BUSY
			*The next state changes to CHANNEL_IDLE, CHIP_IDLE
			*******************************************************************************************************/
			sub->current_time = ssd->current_time;
			sub->current_state = SR_W_TRANSFER;
			sub->next_state = SR_COMPLETE;
			sub->next_state_predict_time = ssd->current_time + 7 * ssd->parameter->time_characteristics.tWC + (sub->size*ssd->parameter->subpage_capacity)*ssd->parameter->time_characteristics.tWC;
			sub->complete_time = sub->next_state_predict_time;
			time = sub->complete_time;

			ssd->channel_head[location->channel].current_state = CHANNEL_TRANSFER;
			ssd->channel_head[location->channel].current_time = ssd->current_time;
			ssd->channel_head[location->channel].next_state = CHANNEL_IDLE;
			ssd->channel_head[location->channel].next_state_predict_time = time;

			ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_WRITE_BUSY;
			ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_IDLE;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = time + ssd->parameter->time_characteristics.tPROG;

			break;
		}
		default:  return ERROR;

		}//switch(aim_state)	
	}//if(command==NORMAL)

	else if (command == TWO_PLANE)
	{
		/**********************************************************************************************
		*Advanced order TWO_PLANE processing, where the TWO_PLANE advanced command is a high-level command to read the child request
		*State transition and ordinary command, the difference is in SR_R_C_A_TRANSFER when the calculation of time is serial, because the sharing of a channel channel
		*Also SR_R_DATA_TRANSFER also share a channel
		**********************************************************************************************/
		if ((sub1 == NULL) || (sub2 == NULL))
		{
			return ERROR;
		}
		sub_twoplane_one = sub1;
		sub_twoplane_two = sub2;
		location = sub1->location;

		switch (aim_state)
		{
		case SR_R_C_A_TRANSFER:
		{
			sub_twoplane_one->current_time = ssd->current_time;
			sub_twoplane_one->current_state = SR_R_C_A_TRANSFER;
			sub_twoplane_one->next_state = SR_R_READ;
			sub_twoplane_one->next_state_predict_time = ssd->current_time + 14 * ssd->parameter->time_characteristics.tWC;
			sub_twoplane_one->begin_time = ssd->current_time;

			ssd->channel_head[sub_twoplane_one->location->channel].chip_head[sub_twoplane_one->location->chip].die_head[sub_twoplane_one->location->die].plane_head[sub_twoplane_one->location->plane].add_reg_ppn = sub_twoplane_one->ppn;
			ssd->read_count++;
			ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_read_count++;

			sub_twoplane_two->current_time = ssd->current_time;
			sub_twoplane_two->current_state = SR_R_C_A_TRANSFER;
			sub_twoplane_two->next_state = SR_R_READ;
			sub_twoplane_two->next_state_predict_time = sub_twoplane_one->next_state_predict_time;
			sub_twoplane_two->begin_time = ssd->current_time;

			ssd->channel_head[sub_twoplane_two->location->channel].chip_head[sub_twoplane_two->location->chip].die_head[sub_twoplane_two->location->die].plane_head[sub_twoplane_two->location->plane].add_reg_ppn = sub_twoplane_two->ppn;
			ssd->read_count++;
			ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_read_count++;
			ssd->m_plane_read_count++;

			ssd->channel_head[location->channel].current_state = CHANNEL_C_A_TRANSFER;
			ssd->channel_head[location->channel].current_time = ssd->current_time;
			ssd->channel_head[location->channel].next_state = CHANNEL_IDLE;
			ssd->channel_head[location->channel].next_state_predict_time = ssd->current_time + 14 * ssd->parameter->time_characteristics.tWC;

			ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_C_A_TRANSFER;
			ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_READ_BUSY;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = ssd->current_time + 14 * ssd->parameter->time_characteristics.tWC;


			break;
		}
		case SR_R_DATA_TRANSFER:
		{
			sub_twoplane_one->current_time = ssd->current_time;
			sub_twoplane_one->current_state = SR_R_DATA_TRANSFER;
			sub_twoplane_one->next_state = SR_COMPLETE;
			sub_twoplane_one->next_state_predict_time = ssd->current_time + (sub_twoplane_one->size*ssd->parameter->subpage_capacity)*ssd->parameter->time_characteristics.tRC;
			sub_twoplane_one->complete_time = sub_twoplane_one->next_state_predict_time;

			sub_twoplane_two->current_time = sub_twoplane_one->next_state_predict_time;
			sub_twoplane_two->current_state = SR_R_DATA_TRANSFER;
			sub_twoplane_two->next_state = SR_COMPLETE;
			sub_twoplane_two->next_state_predict_time = sub_twoplane_two->current_time + (sub_twoplane_two->size*ssd->parameter->subpage_capacity)*ssd->parameter->time_characteristics.tRC;
			sub_twoplane_two->complete_time = sub_twoplane_two->next_state_predict_time;


			if (sub_twoplane_one->update_read_flag == 1)
				sub_twoplane_one->update_read_flag = 0;

			if (sub_twoplane_two->update_read_flag == 1)
				sub_twoplane_two->update_read_flag = 0;

			ssd->channel_head[location->channel].current_state = CHANNEL_DATA_TRANSFER;
			ssd->channel_head[location->channel].current_time = ssd->current_time;
			ssd->channel_head[location->channel].next_state = CHANNEL_IDLE;
			ssd->channel_head[location->channel].next_state_predict_time = sub_twoplane_one->next_state_predict_time;

			ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_DATA_TRANSFER;
			ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_IDLE;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = sub_twoplane_one->next_state_predict_time;

			//Read state conversion is completed, then plane register value is set to the initial value
			ssd->channel_head[sub_twoplane_one->location->channel].chip_head[sub_twoplane_one->location->chip].die_head[sub_twoplane_one->location->die].plane_head[sub_twoplane_one->location->plane].add_reg_ppn = -1;
			ssd->channel_head[sub_twoplane_two->location->channel].chip_head[sub_twoplane_two->location->chip].die_head[sub_twoplane_two->location->die].plane_head[sub_twoplane_two->location->plane].add_reg_ppn = -1;

			break;
		}
		default:  return ERROR;
		}//switch(aim_state)	
	}//else if(command==TWO_PLANE)
	else
	{
		printf("\nERROR: Unexpected command !\n");
		return ERROR;
	}

	return SUCCESS;
}