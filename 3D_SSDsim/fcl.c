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


/**************************************************************************
*This function is also only deal with read requests, processing chip current state is CHIP_WAIT,
*Or the next state is CHIP_DATA_TRANSFER and the next state of the expected time is less than the current time of the chip
***************************************************************************/
Status services_2_r_data_trans(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag)
{
	unsigned int chip = 0;
	unsigned int sub_r_count, i = 0;
	unsigned int aim_die;

	struct sub_request ** sub_r_request = NULL;
	sub_r_request = (struct sub_request **)malloc(ssd->parameter->plane_die * sizeof(struct sub_request *));
	alloc_assert(sub_r_request, "sub_r_request");
	for (i = 0; i < ssd->parameter->plane_die; i++)
	{
		sub_r_request[i] = NULL;
	}

	for (chip = 0; chip<ssd->channel_head[channel].chip; chip++)
	{
		if ((ssd->channel_head[channel].chip_head[chip].current_state == CHIP_DATA_TRANSFER) ||
			((ssd->channel_head[channel].chip_head[chip].next_state == CHIP_DATA_TRANSFER) &&
			(ssd->channel_head[channel].chip_head[chip].next_state_predict_time <= ssd->current_time)))
		{
			for (aim_die = 0; aim_die < ssd->parameter->die_chip; aim_die++)
			{
				if ((ssd->parameter->advanced_commands&AD_TWOPLANE_READ) == AD_TWOPLANE_READ)
				{
	                sub_r_count = find_read_sub_request(ssd, channel, chip, aim_die, sub_r_request, SR_R_DATA_TRANSFER, TWO_PLANE);
					if (sub_r_count > 1)
					{
						go_one_step(ssd, sub_r_request, sub_r_count, SR_R_DATA_TRANSFER, TWO_PLANE);
						*change_current_time_flag = 0;
						*channel_busy_flag = 1;
						break;
					}
					else
					{
						sub_r_count = find_read_sub_request(ssd, channel, chip, aim_die, sub_r_request, SR_R_DATA_TRANSFER, NORMAL);
						if (sub_r_count == 1)
						{
							go_one_step(ssd, sub_r_request, sub_r_count, SR_R_DATA_TRANSFER, NORMAL);
							*change_current_time_flag = 0;
							*channel_busy_flag = 1;
							break;
						}
					}
				}
			}
		}

		if (*channel_busy_flag == 1)
			break;
	}
	for (i = 0; i < ssd->parameter->plane_die; i++)
		sub_r_request[i] = NULL;
	free(sub_r_request);
	sub_r_request = NULL;
	return SUCCESS;
}

Status services_2_r_read(struct ssd_info * ssd)
{
	unsigned int i,j,subs_count = 0,aim_die;

	struct sub_request ** subs = NULL;
	subs = (struct sub_request **)malloc((ssd->parameter->plane_die) * sizeof(struct sub_request *));
	alloc_assert(subs, "subs");
	for (i = 0; i < ssd->parameter->plane_die; i++)
	{
		subs[i] = NULL;
	}

	for (i = 0; i < ssd->parameter->channel_number; i++)                                    
	{
		for (j = 0; j < ssd->parameter->chip_channel[i]; j++)
		{
			if ((ssd->channel_head[i].chip_head[j].current_state == CHIP_READ_BUSY) ||
				((ssd->channel_head[i].chip_head[j].next_state == CHIP_READ_BUSY) &&
				(ssd->channel_head[i].chip_head[j].next_state_predict_time <= ssd->current_time)))
			{
				for (aim_die = 0; aim_die < ssd->parameter->die_chip; aim_die++)
				{
					//首先去找mutli plane请求，能找到进行,否则进行normal plane
					subs_count = find_read_sub_request(ssd, i, j, aim_die, subs, SR_R_READ, TWO_PLANE);
					if (subs_count > 1)
					{
						go_one_step(ssd, subs, subs_count, SR_R_READ, TWO_PLANE);
						break;
					}
					else
					{
						subs_count = find_read_sub_request(ssd, i, j, aim_die, subs, SR_R_READ, NORMAL);
						if (subs_count == 1)
						{
							go_one_step(ssd, subs, subs_count, SR_R_READ, NORMAL);
							break;
						}
					}
				}
			}
		}
	}

	for (i = 0; i < ssd->parameter->plane_die; i++)
		subs[i] = NULL;
	free(subs);
	subs = NULL;

	return SUCCESS;
}

/**************************************************************************************
*Function function is given in the channel, chip, die above looking for reading requests
*The request for this child ppn corresponds to the ppn of the corresponding plane's register
*****************************************************************************************/
unsigned int find_read_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request ** subs, unsigned int state, unsigned int command)
{
	struct sub_request * sub = NULL;
	unsigned int add_reg, j = 0;

	sub = ssd->channel_head[channel].subs_r_head;
	j = 0;

	
	while (sub != NULL)
	{
		if (sub->location->chip == chip && sub->location->die == die)
		{
			if (sub->next_state == state && sub->next_state_predict_time <= ssd->current_time)
			{
				if (command == TWO_PLANE)
				{
					if (sub->mutliplane_flag == 1)
					{
						add_reg = ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].die_head[sub->location->die].plane_head[sub->location->plane].add_reg_ppn;
						if (sub->ppn == add_reg)
						{
							subs[j] = sub;
							j++;
						}
						else
						{
							printf("error add_reg is wrong!\n");
							getchar();
						}
					}
				}
				else if (command == NORMAL)
				{
					if (sub->mutliplane_flag == 0)
					{
						add_reg = ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].die_head[sub->location->die].plane_head[sub->location->plane].add_reg_ppn;
						if (sub->ppn == add_reg)
						{
							subs[j] = sub;
							j++;
						}
						else
						{
							printf("error add_reg is wrong!\n");
							getchar();
						}
					}
				}
			}
		}

		if (command == TWO_PLANE)
		{
			if (j == ssd->parameter->plane_die)
				break;
		}
		else
		{
			if (j == 1)
				break;
		}
		sub = sub->next_node;
	}
 

	if (j > ssd->parameter->plane_die)
	{
		printf("error,beyong plane_die\n");
		getchar();
	}

	return j;
}

/*********************************************************************************************
* function that specifically serves a read request
*1，Only when the current state of the sub request is SR_R_C_A_TRANSFER
*2，The current state of the read request is SR_COMPLETE or the next state is SR_COMPLETE and 
*the next state arrives less than the current time
**********************************************************************************************/
Status services_2_r_complete(struct ssd_info * ssd)
{
	unsigned int i = 0;
	struct sub_request * sub = NULL, *p = NULL;
	
	for (i = 0; i<ssd->parameter->channel_number; i++)                                       /*This loop does not require the channel time, when the read request is completed, it will be removed from the channel queue*/
	{
		sub = ssd->channel_head[i].subs_r_head;
		while (sub != NULL)
		{
			if ((sub->current_state == SR_COMPLETE) || ((sub->next_state == SR_COMPLETE) && (sub->next_state_predict_time <= ssd->current_time)))
			{
				if (sub != ssd->channel_head[i].subs_r_head)                             /*if the request is completed, we delete it from read queue */
				{
					p->next_node = sub->next_node;
					if (sub == ssd->channel_head[i].subs_r_tail)
						ssd->channel_head[i].subs_r_tail = p;
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


/*****************************************************************************************
*This function is also a service that only reads the child request, and is in a wait state
******************************************************************************************/
Status services_2_r_wait(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag)
{
	struct sub_request ** sub_mutliplane_place = NULL;
	unsigned int sub_r_req_count, i ,chip;

	sub_mutliplane_place = (struct sub_request **)malloc(ssd->parameter->plane_die * sizeof(struct sub_request *));
	alloc_assert(sub_mutliplane_place, "sub_mutliplane_place");
	for (i = 0; i < ssd->parameter->plane_die; i++)
	{
		sub_mutliplane_place[i] = NULL;
	}

	if ((ssd->parameter->advanced_commands&AD_TWOPLANE_READ) == AD_TWOPLANE_READ)         /*to find whether there are two sub request can be served by two plane operation*/
	{
		for (chip = 0; chip < ssd->parameter->chip_channel[channel]; chip++)
		{
			sub_r_req_count = find_mutliplane_sub_request(ssd, channel, chip, sub_mutliplane_place, TWO_PLANE);
			if (sub_r_req_count == 0)
			{
				//printf("sub_r_req in null");
				*channel_busy_flag = 0;
				for (i = 0; i < ssd->parameter->plane_die; i++)
				{
					sub_mutliplane_place[i] = NULL;
				}
				free(sub_mutliplane_place);
				sub_mutliplane_place = NULL;
				return FAILURE;
			}
			else if (sub_r_req_count == 1)
			{
				go_one_step(ssd, sub_mutliplane_place, sub_r_req_count, SR_R_C_A_TRANSFER, NORMAL);
				*change_current_time_flag = 0;
				*channel_busy_flag = 1;
			}
			else
			{
				go_one_step(ssd, sub_mutliplane_place, sub_r_req_count, SR_R_C_A_TRANSFER, TWO_PLANE);
				*change_current_time_flag = 0;
				*channel_busy_flag = 1;
			}
		}
	}

	for (i = 0; i < ssd->parameter->plane_die; i++)
	{
		sub_mutliplane_place[i] = NULL;
	}
	free(sub_mutliplane_place);
	sub_mutliplane_place = NULL;

	return SUCCESS;
}

/*************************************************************************
*In dealing with the sub-request request advanced command, the use of this
*or find the implementation of advanced orders sub_request
**************************************************************************/
unsigned int find_mutliplane_sub_request(struct ssd_info * ssd, unsigned int channel, unsigned int chip, struct sub_request ** sub_mutliplane_place, unsigned int command)
{
	unsigned int i = 0, j = 0, plane_flag;
	struct sub_request * sub_plane_request = NULL;

	i = 0;
	sub_plane_request = ssd->channel_head[channel].subs_r_head;
	while (sub_plane_request != NULL)
	{
		if (sub_plane_request->current_state == SR_WAIT)
		{
			if (sub_plane_request->location->chip == chip)
			{
				if (i == 0)
				{
					if (((ssd->channel_head[sub_plane_request->location->channel].chip_head[sub_plane_request->location->chip].current_state == CHIP_IDLE) ||
						((ssd->channel_head[sub_plane_request->location->channel].chip_head[sub_plane_request->location->chip].next_state == CHIP_IDLE) &&
						(ssd->channel_head[sub_plane_request->location->channel].chip_head[sub_plane_request->location->chip].next_state_predict_time <= ssd->current_time))))
					{
						sub_mutliplane_place[0] = sub_plane_request;
						i++;
					}
				}
				else
				{
					if ((sub_mutliplane_place[0]->location->chip == sub_plane_request->location->chip) &&
						(sub_mutliplane_place[0]->location->die == sub_plane_request->location->die) &&
						(sub_mutliplane_place[0]->location->page == sub_plane_request->location->page))
					{
						plane_flag = 0;
						for (j = 0; j < i; j++)
						{
							if (sub_mutliplane_place[j]->location->plane == sub_plane_request->location->plane)
								plane_flag = 1;
						}
						if (plane_flag == 0)
						{
							sub_mutliplane_place[i] = sub_plane_request;
							i++;
						}
						
						/*
						if (sub_mutliplane_place[0]->location->plane != sub_plane_request->location->plane)
						{
							sub_mutliplane_place[i] = sub_plane_request;
							i++;
						}
						*/
					}
				}
				if (command == TWO_PLANE)
				{
					if (i == ssd->parameter->plane_die)
					{
						break;
					}
				}
				else
				{
					if (i == 1)
						break;
				}
			}
		}

		sub_plane_request = sub_plane_request->next_node;
	}

	if (i > ssd->parameter->plane_die)
	{
		printf("error,beyong plane_die\n");
		getchar();
	}

	return i;
}

/***********************************************************************************************************
*1.The state transition of the child request, and the calculation of the time, are handled by this function
*2.The state of the execution of the normal command, and the calculation of the time, are handled by this function
****************************************************************************************************************/
Status go_one_step(struct ssd_info * ssd, struct sub_request ** subs, unsigned int subs_count, unsigned int aim_state, unsigned int command)
{
	unsigned int i = 0, j = 0, k = 0, m = 0;
	long long time = 0;

	struct sub_request * sub = NULL;
	struct local * location = NULL;

	for (i = 0; i < subs_count; i++)
	{
		if (subs[i] == NULL)
		{
			printf("ERROR! no subs state jump\n");
			getchar();
			return ERROR;
		}
	}

	/***************************************************************************************************
	*When dealing with ordinary commands, the target state of the read request is divided into the following
	*cases: SR_R_READ, SR_R_C_A_TRANSFER, SR_R_DATA_TRANSFER
	*
	*The target status of the write request is only SR_W_TRANSFER
	****************************************************************************************************/
	if (command == NORMAL)
	{
		sub = subs[0];
		location = subs[0]->location;

		switch (aim_state)
		{
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

			ssd->read_count++;
			ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].blk_head[location->block].page_read_count++;

			ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_READ_BUSY;
			ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_DATA_TRANSFER;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = ssd->current_time + ssd->parameter->time_characteristics.tR;

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

			ssd->channel_head[location->channel].chip_head[location->chip].die_head[location->die].plane_head[location->plane].add_reg_ppn = -1;

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
		}
	}
	else if (command == TWO_PLANE)
	{
		/**********************************************************************************************
		*Advanced order TWO_PLANE processing, where the TWO_PLANE advanced command is a high-level command to read the child request
		*State transition and ordinary command, the difference is in SR_R_C_A_TRANSFER when the calculation of time is serial, because the sharing of a channel channel
		*Also SR_R_DATA_TRANSFER also share a channel
		**********************************************************************************************/
		location = subs[0]->location;

		switch (aim_state)
		{
			case SR_R_C_A_TRANSFER:
			{
				for (i = 0; i < subs_count; i++)
				{
					//更新子请求的时间线，地址的传输是串行的
					if (i == 0)
						subs[i]->current_time = ssd->current_time;
					else
						subs[i]->current_time = subs[i - 1]->next_state_predict_time;

					subs[i]->current_state = SR_R_C_A_TRANSFER;
					subs[i]->next_state = SR_R_READ;
					subs[i]->next_state_predict_time = subs[i]->current_time + 7 * ssd->parameter->time_characteristics.tWC;
					subs[i]->begin_time = ssd->current_time;

					//更新地址寄存器
					ssd->channel_head[subs[i]->location->channel].chip_head[subs[i]->location->chip].die_head[subs[i]->location->die].plane_head[subs[i]->location->plane].add_reg_ppn = subs[i]->ppn;    //将要写入的地址传送到地址寄存器

					//设置请求类型
					subs[i]->mutliplane_flag = 1;
				}
				i--;
				//更新channel/chip的时间线
				ssd->channel_head[location->channel].current_state = CHANNEL_C_A_TRANSFER;
				ssd->channel_head[location->channel].current_time = ssd->current_time;
				ssd->channel_head[location->channel].next_state = CHANNEL_IDLE;
				ssd->channel_head[location->channel].next_state_predict_time = subs[i]->next_state_predict_time;   //muitli plane 传地址共用一个channel通道，此地址传输的时间是串行的

				ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_C_A_TRANSFER;
				ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
				ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_READ_BUSY;
				ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = subs[i]->next_state_predict_time;


				break;
			}
			case SR_R_READ:
			{
				//更新子请求的时间线
				for (i = 0; i < subs_count; i++)
				{
					subs[i]->current_time = ssd->current_time;
					subs[i]->current_state = SR_R_READ;
					subs[i]->next_state = SR_R_DATA_TRANSFER;
					subs[i]->next_state_predict_time = subs[i]->current_time + ssd->parameter->time_characteristics.tR;

					//更新读操作的计数值
					ssd->channel_head[subs[i]->location->channel].chip_head[subs[i]->location->chip].die_head[subs[i]->location->die].plane_head[subs[i]->location->plane].blk_head[subs[i]->location->block].page_read_count++;    //read操作计数值增加
					ssd->read_count++;
				}
				ssd->m_plane_read_count++;
				i--;
				//更新chip的时间线
				ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_READ_BUSY;
				ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
				ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_DATA_TRANSFER;
				ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = ssd->current_time + ssd->parameter->time_characteristics.tR;

				break;
			}
			case SR_R_DATA_TRANSFER:
			{
				//更新子请求的时间线
				for (i = 0; i < subs_count; i++)
				{
					if (i == 0)
						subs[i]->current_time = ssd->current_time;
					else
						subs[i]->current_time = subs[i - 1]->next_state_predict_time;

					subs[i]->current_state = SR_R_DATA_TRANSFER;
					subs[i]->next_state = SR_COMPLETE;
					subs[i]->next_state_predict_time = subs[i]->current_time + (subs[i]->size*ssd->parameter->subpage_capacity)*ssd->parameter->time_characteristics.tRC;
					subs[i]->complete_time = subs[i]->next_state_predict_time;

					//将地址寄存器清空
					ssd->channel_head[subs[i]->location->channel].chip_head[subs[i]->location->chip].die_head[subs[i]->location->die].plane_head[subs[i]->location->plane].add_reg_ppn = -1;

					//pre read完成，更新完成的标志位
					if (subs[i]->update_read_flag == 1)
						subs[i]->update_read_flag = 0;
				}
				i--;
				//更新channel/chip的时间线
				ssd->channel_head[location->channel].current_state = CHANNEL_DATA_TRANSFER;
				ssd->channel_head[location->channel].current_time = ssd->current_time;
				ssd->channel_head[location->channel].next_state = CHANNEL_IDLE;
				ssd->channel_head[location->channel].next_state_predict_time = subs[i]->next_state_predict_time;

				ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_DATA_TRANSFER;
				ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
				ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_IDLE;
				ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = subs[i]->next_state_predict_time;
				break;
			}
			default:  return ERROR;
		}
	}
	else
	{
		printf("\nERROR: Unexpected command !\n");
		return ERROR;
	}

	return SUCCESS;
}

/****************************************
Write the request function of the request
*****************************************/
Status services_2_write(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag)
{
	int j = 0;
	unsigned int chip_token = 0;
	long long time = 0;

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
	unsigned int plane_count = 0;                                                                                                                       /*record which plane has sub request in static allocation*/
	struct sub_request *sub = NULL, *p = NULL;
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
		//超过更新队列深度，将trace文件读取阻塞
		if (update_count > ssd->parameter->update_reqeust_max)
		{
			printf("update sub request is full!\n");
			ssd->buffer_full_flag = 1;  //blcok the buffer
		}
		else
			ssd->buffer_full_flag = 0;

		if (subs_count >= ssd->parameter->plane_die)
		{	
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
				//后续高级命令的添加
				for (i = 1; i<subs_count; i++)
				{
					subs[i] = NULL;
				}
				subs_count = 1;
				get_ppn_for_normal_command(ssd, channel, chip, subs[0]);
				printf("lz:normal_wr_1\n");
				getchar();
			}
		}
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

/******************************************************************************************************
*The function of the function is to find two pages of the same horizontal position for the two plane 
*command, and modify the statistics, modify the status of the page
*******************************************************************************************************/
Status find_level_page(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request **sub, unsigned int subs_count)
{
	unsigned int i, aim_page = 0, old_plane;
	struct gc_operation *gc_node;
	unsigned int gc_add;

	unsigned int plane,active_block, page,equal_flag;
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

