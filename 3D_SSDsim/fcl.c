/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName： ssd.c
Author: Zuo Lu 		Version: 1.0	Date:2017/04/06
Description:
fcl layer: remove other high-level commands, leaving only mutli plane;

History:
<contributor>     <time>        <version>       <desc>                   <e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim       617376665@qq.com

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

extern int plane_cmplt;
extern int buffer_full_flag;
/******************************************************
*函数的功能是在给出的channel，chip，die上面寻找读子请求
*这个子请求的ppn要与相应的plane的寄存器里面的ppn相符
*******************************************************/
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
*函数的功能是寻找写子请求。
*分两种情况1，要是是完全动态分配就在ssd->subs_w_head队列上找
*2，要是不是完全动态分配那么就在ssd->channel_head[channel].subs_w_head队列上查找
********************************************************************************/
struct sub_request * find_write_sub_request(struct ssd_info * ssd, unsigned int channel)
{
	struct sub_request * sub = NULL, *p = NULL;
	if ((ssd->parameter->allocation_scheme == 0) && (ssd->parameter->dynamic_allocation == 0))    /*是完全的动态分配*/
	{
		sub = ssd->subs_w_head;
		while (sub != NULL)
		{
			if (sub->current_state == SR_WAIT)
			{
				if (sub->update != NULL)                                                      /*如果有需要提前读出的页*/
				{
					if ((sub->update->current_state == SR_COMPLETE) || ((sub->update->next_state == SR_COMPLETE) && (sub->update->next_state_predict_time <= ssd->current_time)))   //被更新的页已经被读出
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

		if (sub == NULL)                                                                      /*如果没有找到可以服务的子请求，跳出这个for循环*/
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
*专门为读子请求服务的函数
*1，只有当读子请求的当前状态是SR_R_C_A_TRANSFER
*2，读子请求的当前状态是SR_COMPLETE或者下一状态是SR_COMPLETE并且下一状态到达的时间比当前时间小
**********************************************************************************************/
Status services_2_r_cmd_trans_and_complete(struct ssd_info * ssd)
{
	unsigned int i = 0;
	struct sub_request * sub = NULL, *p = NULL;
	

	for (i = 0; i<ssd->parameter->channel_number; i++)                                       /*这个循环处理不需要channel的时间(读命令已经到达chip，chip由ready变为busy)，当读请求完成时，将其从channel的队列中取出*/
	{
		sub = ssd->channel_head[i].subs_r_head;

		while (sub != NULL)
		{
			if (sub->current_state == SR_R_C_A_TRANSFER)                                  /*读命令发送完毕，将对应的die置为busy，同时修改sub的状态; 这个部分专门处理读请求由当前状态为传命令变为die开始busy，die开始busy不需要channel为空，所以单独列出*/
			{
				if (sub->next_state_predict_time <= ssd->current_time)
				{
					go_one_step(ssd, sub, NULL, SR_R_READ, NORMAL);                      /*状态跳变处理函数*/

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
*这个函数也是只处理读子请求，处理chip当前状态是CHIP_WAIT，
*或者下一个状态是CHIP_DATA_TRANSFER并且下一状态的预计时间小于当前时间的chip
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
				sub = find_read_sub_request(ssd, channel, chip, die);                   /*在channel,chip,die中找到读子请求*/
				if (sub != NULL)
				{
					break;
				}
			}
			if (sub == NULL)
			{
				continue;
			} 

			/**************************************************************************************
			*如果ssd支持高级命令，那没我们可以一起处理支持AD_TWOPLANE_READ，AD_INTERLEAVE的读子请求
			*1，有可能产生了two plane操作，在这种情况下，将同一个die上的两个plane的数据依次传出
			*2，有可能产生了interleave操作，在这种情况下，将不同die上的两个plane的数据依次传出
			***************************************************************************************/
			if ((ssd->parameter->advanced_commands&AD_TWOPLANE_READ) == AD_TWOPLANE_READ)				/*有可能产生了two plane操作，在这种情况下，将同一个die上的两个plane的数据依次传出*/
			{
				sub_twoplane_one = sub;
				sub_twoplane_two = NULL;
				/*为了保证找到的sub_twoplane_two与sub_twoplane_one不同，令add_reg_ppn=-1*/
				ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[sub->location->plane].add_reg_ppn = -1;
				sub_twoplane_two = find_read_sub_request(ssd, channel, chip, die);               /*在相同的channel,chip,die中寻找另外一个读子请求*/

				/******************************************************
				*如果找到了那么就执行TWO_PLANE的状态转换函数go_one_step
				*如果没找到那么就执行普通命令的状态转换函数go_one_step
				******************************************************/
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
			else                                                                                 /*如果ssd不支持高级命令那么就执行一个一个的执行读子请求*/
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

	if (*channel_busy_flag == 0)
		//printf("\n");

	return SUCCESS;
}


/******************************************************
*这个函数也是只服务读子请求，并且处于等待状态的读子请求
*******************************************************/
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
		/*寻找能执行two_plane的两个读子请求*/
		find_interleave_twoplane_sub_request(ssd, channel, &sub_twoplane_one, &sub_twoplane_two, TWO_PLANE);

		//find_interleave_twoplane_sub_request(ssd, channel, sub_twoplane_one, sub_twoplane_two, TWO_PLANE);

		if (sub_twoplane_two != NULL)                                                     /*可以执行two plane read 操作*/
		{

			go_one_step(ssd, sub_twoplane_one, sub_twoplane_two, SR_R_C_A_TRANSFER, TWO_PLANE);
			*change_current_time_flag = 0;
			*channel_busy_flag = 1;                                                       /*已经占用了这个周期的总线，不用执行die中数据的回传*/
		}
		else if ((ssd->parameter->advanced_commands&AD_INTERLEAVE) != AD_INTERLEAVE)       /*没有满足条件的两个page，，并且没有interleave read命令时，只能执行单个page的读*/
		{
			while (sub != NULL)                                                            /*if there are read requests in queue, send one of them to target die*/
			{
				if (sub->current_state == SR_WAIT)
				{	   
					/*注意下个这个判断条件与services_2_r_data_trans中判断条件的不同*/
					if ((ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].current_state == CHIP_IDLE) || ((ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].next_state == CHIP_IDLE) &&
						(ssd->channel_head[sub->location->channel].chip_head[sub->location->chip].next_state_predict_time <= ssd->current_time)))
					{
						go_one_step(ssd, sub, NULL, SR_R_C_A_TRANSFER, NORMAL);

						*change_current_time_flag = 0;
						*channel_busy_flag = 1;                                           /*已经占用了这个周期的总线，不用执行die中数据的回传*/
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

		if (*channel_busy_flag == 0)
		{
			//printf("chip busy,%d\n",channel);
		}

	}

	/*******************************
	*ssd不能执行执行高级命令的情况下
	*******************************/
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
					*channel_busy_flag = 1;                                              /*已经占用了这个周期的总线，不用执行die中数据的回传*/
					break;
				}
				else
				{
					/*因为die的busy导致的阻塞*/
				}
			}
			sub = sub->next_node;
		}
	}

	return SUCCESS;
}

/*********************************************************************
*当一个写子请求处理完后，要从请求队列上删除，这个函数就是执行这个功能。
**********************************************************************/
int delete_w_sub_request(struct ssd_info * ssd, unsigned int channel, struct sub_request * sub)
{
	struct sub_request * p = NULL;
	if (sub == ssd->channel_head[channel].subs_w_head)                                   /*将这个子请求从channel队列中删除*/
	{
		if (ssd->channel_head[channel].subs_w_head != ssd->channel_head[channel].subs_w_tail)
		{
			ssd->channel_head[channel].subs_w_head = sub->next_node;
		}
		else
		{
			ssd->channel_head[channel].subs_w_head = NULL;
			ssd->channel_head[channel].subs_w_tail = NULL;
		}
	}
	else
	{
		p = ssd->channel_head[channel].subs_w_head;
		while (p->next_node != sub)
		{
			p = p->next_node;
		}

		if (sub->next_node != NULL)
		{
			p->next_node = sub->next_node;
		}
		else
		{
			p->next_node = NULL;
			ssd->channel_head[channel].subs_w_tail = p;
		}
	}

	return SUCCESS;
}




/********************
写子请求的处理函数
*********************/
Status services_2_write(struct ssd_info * ssd, unsigned int channel, unsigned int * channel_busy_flag, unsigned int * change_current_time_flag)
{
	int j = 0, chip = 0;
	unsigned int k = 0;
	unsigned int  old_ppn = 0, new_ppn = 0;
	unsigned int chip_token = 0, die_token = 0;
	//unsigned int  die = 0, plane = 0;
	long long time = 0;
	struct sub_request * sub = NULL, *p = NULL;
	struct sub_request * sub_twoplane_one = NULL, *sub_twoplane_two = NULL;

	/************************************************************************************************************************
	*由于是动态分配，所有的写子请求挂在ssd->subs_w_head，即分配前不知道写在哪个channel上
	*************************************************************************************************************************/
	if (ssd->subs_w_head != NULL)
	{
		if (ssd->parameter->allocation_scheme == 0)                                       /*动态分配*/
		{
			for (j = 0; j<ssd->channel_head[channel].chip; j++)							  //遍历所有的chip
			{
				if (ssd->subs_w_head == NULL)											  //写请求处理完即停止循环
				{
					break;
				}

				chip_token = ssd->channel_head[channel].token;                            /*令牌*/
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
							*channel_busy_flag = 1;                                 /*执行了一个请求，传输了数据，占用了总线，需要跳出到下一个channel*/
							//ssd->channel_head[channel].chip_head[chip_token].token = (ssd->channel_head[channel].chip_head[chip_token].token + 1) % ssd->parameter->die_chip;
							ssd->channel_head[channel].token = (ssd->channel_head[channel].token + 1) % ssd->parameter->chip_channel[channel];
							break;
						}
					}
				}
				ssd->channel_head[channel].token = (ssd->channel_head[channel].token + 1) % ssd->parameter->chip_channel[channel];  //更新chip
			}
		}
	}
	return SUCCESS;
}




/****************************************************************************************************************************
*当ssd支持高级命令时，这个函数的作用就是处理高级命令的写子请求
*根据请求的个数，决定选择哪种高级命令（这个函数只处理写请求，读请求已经分配到每个channel，所以在执行时之间进行选取相应的命令）
*****************************************************************************************************************************/
struct ssd_info *dynamic_advanced_process(struct ssd_info *ssd, unsigned int channel, unsigned int chip)
{
	//unsigned int die = 0, plane = 0;
	unsigned int subs_count = 0;
	unsigned int plane_count = 0;
	int flag;                                                                   /*record the max subrequest that can be executed in the same channel. it will be used when channel-level priority order is highest and allocation scheme is full dynamic allocation*/
	unsigned int plane_place;                                                             /*record which plane has sub request in static allocation*/
	struct sub_request *sub = NULL, *p = NULL, *sub0 = NULL, *sub1 = NULL, *sub2 = NULL, *sub3 = NULL, *sub0_rw = NULL, *sub1_rw = NULL, *sub2_rw = NULL, *sub3_rw = NULL;
	struct sub_request ** subs = NULL;
	unsigned int max_sub_num = 0;
	unsigned int die_token = 0, plane_token = 0;
	unsigned int * plane_bits = NULL;
	unsigned int interleaver_count = 0;

	unsigned int mask = 0x00000001;
	unsigned int i = 0, j = 0;

	plane_count = ssd->parameter->plane_die;
	//ssd->real_time_subreq = 0;
	max_sub_num = (ssd->parameter->die_chip)*(ssd->parameter->plane_die);    //这里考虑到了chip die之间的并行性，故max_sub_num表示一个chip上最多同时执行的请求数
	//gate = max_sub_num;
	subs = (struct sub_request **)malloc(max_sub_num*sizeof(struct sub_request *));
	alloc_assert(subs, "sub_request");

	for (i = 0; i<max_sub_num; i++)
	{
		subs[i] = NULL;  //可执行请求数组
	}

	if ((ssd->parameter->allocation_scheme == 0))                                           /*全动态分配，需要从ssd->subs_w_head上选取等待服务的子请求*/
	{
		if (ssd->parameter->dynamic_allocation == 0)
		{
			sub = ssd->subs_w_head;
		}
		else
		{
			sub = ssd->channel_head[channel].subs_w_head;
		}

		subs_count = 0;

		//while ((sub != NULL) && (subs_count<max_sub_num) && (subs_count<gate))
		while ((sub != NULL) && (subs_count<max_sub_num))
		{
			if (sub->current_state == SR_WAIT)
			{
				if ((sub->update == NULL) || ((sub->update != NULL) && ((sub->update->current_state == SR_COMPLETE) || ((sub->update->next_state == SR_COMPLETE) && (sub->update->next_state_predict_time <= ssd->current_time)))))    //没有需要提前读出的页
				{
					subs[subs_count] = sub;			//将目前状态是wait的子请求放入数组中去
					subs_count++;					//目前处理等待状态的子请求的个数
				}
			}

			p = sub;
			sub = sub->next_node;
		}

		if (subs_count == 0)                                                               /*没有请求可以服务，返回NULL*/
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

		//写子请求不止两个，即可以进行高级命令
		if (subs_count >= 2)
		{	
			/*********************************************
			*two plane,interleave都可以使用
			*在这个channel上，选用interleave_two_plane执行
			**********************************************/
			if (ssd->parameter->dynamic_allocation_priority == 1)						//动态分配方式的选择
			{
				/*
				while (subs_count != 0)
				{
					if ((ssd->parameter->advanced_commands&AD_TWOPLANE) == AD_TWOPLANE && plane_cmplt == 0)
					{
						if (get_ppn_for_advanced_commands(ssd, channel, chip, subs, plane_count, TWO_PLANE) == SUCCESS)
						{
							for (i = 0; i < subs_count - plane_count; i++)
								subs[i] = subs[i + plane_count];
							for (i = subs_count - plane_count; i < subs_count; i++)
								subs[i] = NULL;
							subs_count = subs_count - plane_count;
						}
						else
						{
							for (i = 0; i < subs_count - 1; i++)
								subs[i] = subs[i + 1];
							for (i = subs_count - 1; i < subs_count; i++)
								subs[i] = NULL;
							subs_count = subs_count - 1;
						}
					}
					else
					{
						break;
					}
				}
				*/

				if ((ssd->parameter->advanced_commands&AD_TWOPLANE) == AD_TWOPLANE && plane_cmplt == 0)
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

				}
			}
			else
			{
				if ( (ssd->parameter->advanced_commands&AD_TWOPLANE) == AD_TWOPLANE )
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
				}
			}

		}//if(subs_count>=2)
		else if (subs_count == 1)     //only one request
		{
			get_ppn_for_normal_command(ssd, channel, chip, subs[0]);
			printf("lz:normal_wr_1\n");
		}

	}//if ((ssd->parameter->allocation_scheme==0)) 

	for (i = 0; i<max_sub_num; i++)
	{
		subs[i] = NULL;
	}
	free(subs);
	subs = NULL;
	free(plane_bits);
	return ssd;
}



/***********************************************
*函数的作用是让sub0，sub1的ppn所在的page位置相同
************************************************/
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
	*动态分配的情况下
	*sub1的plane是根据sub0的ssd->channel_head[channel].chip_head[chip].die_head[die].token令牌获取的
	*sub1的channel，chip，die，block，page都和sub0的相同
	************************************************************************************************/
	if (ssd->parameter->allocation_scheme == DYNAMIC_ALLOCATION)
	{
		old_plane_token = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
		for (i = 0; i<ssd->parameter->plane_die; i++)
		{
			plane1 = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
			if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane1].add_reg_ppn == -1)
			{
				find_active_block(ssd, channel, chip, die, plane1);                               /*在plane1中找到活跃块*/
				block1 = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane1].active_block;

				/*********************************************************************************************
				*只有找到的block1与block0相同，才能继续往下寻找相同的page
				*在寻找page时比较简单，直接用last_write_page（上一次写的page）+1就可以了。
				*如果找到的page不相同，那么如果ssd允许贪婪的使用高级命令，这样就可以让小的page 往大的page靠拢
				*********************************************************************************************/
				if (block1 == block0)
				{
					page1 = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane1].blk_head[block1].last_write_page + 1;
					if (page1 == page0)
					{
						break;
					}
					else if (page1<page0)
					{
						if (ssd->parameter->greed_MPW_ad == 1)                                  /*允许贪婪的使用高级命令*/
						{
							//make_same_level(ssd,channel,chip,die,plane1,active_block1,page0); /*小的page地址往大的page地址靠*/
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
			flash_page_state_modify(ssd, sub1, channel, chip, die, plane1, block1, page0);          /*这个函数的作用就是更新page1所对应的物理页以及location还有map表*/
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
*函数的功能是为two plane命令寻找出两个相同水平位置的页，并且修改统计值，修改页的状态
*注意这个函数与上一个函数make_level_page函数的区别，make_level_page这个函数是让sub1与sub0的page位置相同
*而find_level_page函数的作用是在给定的channel，chip，die中找两个位置相同的subA和subB。
*******************************************************************************************************/
Status find_level_page(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, struct sub_request *subA, struct sub_request *subB)
{
	unsigned int i, planeA, planeB, active_blockA, active_blockB, pageA, pageB, aim_page, old_plane;
	struct gc_operation *gc_node;

	old_plane = ssd->channel_head[channel].chip_head[chip].die_head[die].token;

	/************************************************************
	*在动态分配的情况下
	*planeA赋初值为die的令牌，如果planeA是偶数那么planeB=planeA+1
	*planeA是奇数，那么planeA+1变为偶数，再令planeB=planeA+1
	*************************************************************/
	if (ssd->parameter->allocation_scheme == 0)
	{
		planeA = ssd->channel_head[channel].chip_head[chip].die_head[die].token;
		if (planeA % 2 == 0)
		{
			planeB = planeA + 1;
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (ssd->channel_head[channel].chip_head[chip].die_head[die].token + 2) % ssd->parameter->plane_die;
		}
		else
		{
			planeA = (planeA + 1) % ssd->parameter->plane_die;
			planeB = planeA + 1;
			ssd->channel_head[channel].chip_head[chip].die_head[die].token = (ssd->channel_head[channel].chip_head[chip].die_head[die].token + 3) % ssd->parameter->plane_die;
		}
	}
	find_active_block(ssd, channel, chip, die, planeA);                                          //*寻找active_block
	find_active_block(ssd, channel, chip, die, planeB);
	active_blockA = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].active_block;
	active_blockB = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].active_block;

	/*****************************************************
	*如果active_block相同，那么就在这两个块中找相同的page
	*或者使用贪婪的方法找到两个相同的page
	******************************************************/
	if (active_blockA == active_blockB)
	{
		pageA = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[active_blockA].last_write_page + 1;
		pageB = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[active_blockB].last_write_page + 1;
		if (pageA == pageB)                                                                    /*两个可用的页正好在同一个水平位置上*/
		{
			flash_page_state_modify(ssd, subA, channel, chip, die, planeA, active_blockA, pageA);
			flash_page_state_modify(ssd, subB, channel, chip, die, planeB, active_blockB, pageB);
		}
		else
		{
			if (ssd->parameter->greed_MPW_ad == 1)                                             /*贪婪地使用高级命令*/
			{
				if (pageA<pageB)
				{
					aim_page = pageB;
					make_same_level(ssd, channel, chip, die, planeA, active_blockA, aim_page);     /*小的page地址往大的page地址靠*/
				}
				else
				{
					aim_page = pageA;
					make_same_level(ssd, channel, chip, die, planeB, active_blockB, aim_page);
				}
				flash_page_state_modify(ssd, subA, channel, chip, die, planeA, active_blockA, aim_page);
				flash_page_state_modify(ssd, subB, channel, chip, die, planeB, active_blockB, aim_page);
			}
			else                                                                             /*不能贪婪的使用高级命令*/
			{
				subA = NULL;
				subB = NULL;
				ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane;
				return FAILURE;
			}
		}
	}
	/*********************************
	*如果找到的两个active_block不相同
	**********************************/
	else
	{
		pageA = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[active_blockA].last_write_page + 1;
		pageB = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[active_blockB].last_write_page + 1;
		if (pageA<pageB)
		{
			if (ssd->parameter->greed_MPW_ad == 1)                                             /*贪婪地使用高级命令*/
			{
				/*******************************************************************************
				*在planeA中，与active_blockB相同位置的的block中，与pageB相同位置的page是可用的。
				*也就是palneA中的相应水平位置是可用的，将其最为与planeB中对应的页。
				*那么可也让planeA，active_blockB中的page往pageB靠拢
				********************************************************************************/
				if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[active_blockB].page_head[pageB].free_state == PG_SUB)
				{
					make_same_level(ssd, channel, chip, die, planeA, active_blockB, pageB);
					flash_page_state_modify(ssd, subA, channel, chip, die, planeA, active_blockB, pageB);
					flash_page_state_modify(ssd, subB, channel, chip, die, planeB, active_blockB, pageB);
				}
				/********************************************************************************
				*在planeA中，与active_blockB相同位置的的block中，与pageB相同位置的page是可用的。
				*那么就要重新寻找block，需要重新找水平位置相同的一对页
				*********************************************************************************/
				else
				{
					for (i = 0; i<ssd->parameter->block_plane; i++)
					{
						pageA = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[i].last_write_page + 1;
						pageB = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[i].last_write_page + 1;
						if ((pageA<ssd->parameter->page_block) && (pageB<ssd->parameter->page_block))
						{
							if (pageA<pageB)
							{
								if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[i].page_head[pageB].free_state == PG_SUB)
								{
									aim_page = pageB;
									make_same_level(ssd, channel, chip, die, planeA, i, aim_page);
									break;
								}
							}
							else
							{
								if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[i].page_head[pageA].free_state == PG_SUB)
								{
									aim_page = pageA;
									make_same_level(ssd, channel, chip, die, planeB, i, aim_page);
									break;
								}
							}
						}
					}//for (i=0;i<ssd->parameter->block_plane;i++)
					if (i<ssd->parameter->block_plane)
					{
						flash_page_state_modify(ssd, subA, channel, chip, die, planeA, i, aim_page);
						flash_page_state_modify(ssd, subB, channel, chip, die, planeB, i, aim_page);
					}
					else
					{
						subA = NULL;
						subB = NULL;
						ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane;
						return FAILURE;
					}
				}
			}//if (ssd->parameter->greed_MPW_ad==1)  
			else
			{
				subA = NULL;
				subB = NULL;
				ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane;
				return FAILURE;
			}
		}//if (pageA<pageB)
		else
		{
			if (ssd->parameter->greed_MPW_ad == 1)
			{
				if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[active_blockA].page_head[pageA].free_state == PG_SUB)
				{
					make_same_level(ssd, channel, chip, die, planeB, active_blockA, pageA);
					flash_page_state_modify(ssd, subA, channel, chip, die, planeA, active_blockA, pageA);
					flash_page_state_modify(ssd, subB, channel, chip, die, planeB, active_blockA, pageA);
				}
				else
				{
					for (i = 0; i<ssd->parameter->block_plane; i++)
					{
						pageA = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[i].last_write_page + 1;
						pageB = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[i].last_write_page + 1;
						if ((pageA<ssd->parameter->page_block) && (pageB<ssd->parameter->page_block))
						{
							if (pageA<pageB)
							{
								if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[i].page_head[pageB].free_state == PG_SUB)
								{
									aim_page = pageB;
									make_same_level(ssd, channel, chip, die, planeA, i, aim_page);
									break;
								}
							}
							else
							{
								if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[i].page_head[pageA].free_state == PG_SUB)
								{
									aim_page = pageA;
									make_same_level(ssd, channel, chip, die, planeB, i, aim_page);
									break;
								}
							}
						}
					}//for (i=0;i<ssd->parameter->block_plane;i++)
					if (i<ssd->parameter->block_plane)
					{
						flash_page_state_modify(ssd, subA, channel, chip, die, planeA, i, aim_page);
						flash_page_state_modify(ssd, subB, channel, chip, die, planeB, i, aim_page);
					}
					else
					{
						subA = NULL;
						subB = NULL;
						ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane;
						return FAILURE;
					}
				}
			} //if (ssd->parameter->greed_MPW_ad==1) 
			else
			{
				if ((pageA == pageB) && (pageA == 0))
				{
					/*******************************************************************************************
					*下面是两种情况
					*1，planeA，planeB中的active_blockA，pageA位置都可用，那么不同plane 的相同位置，以blockA为准
					*2，planeA，planeB中的active_blockB，pageA位置都可用，那么不同plane 的相同位置，以blockB为准
					********************************************************************************************/
					if ((ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[active_blockA].page_head[pageA].free_state == PG_SUB)
						&& (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[active_blockA].page_head[pageA].free_state == PG_SUB))
					{
						flash_page_state_modify(ssd, subA, channel, chip, die, planeA, active_blockA, pageA);
						flash_page_state_modify(ssd, subB, channel, chip, die, planeB, active_blockA, pageA);
					}
					else if ((ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].blk_head[active_blockB].page_head[pageA].free_state == PG_SUB)
						&& (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].blk_head[active_blockB].page_head[pageA].free_state == PG_SUB))
					{
						flash_page_state_modify(ssd, subA, channel, chip, die, planeA, active_blockB, pageA);
						flash_page_state_modify(ssd, subB, channel, chip, die, planeB, active_blockB, pageA);
					}
					else
					{
						subA = NULL;
						subB = NULL;
						ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane;
						return FAILURE;
					}
				}
				else
				{
					subA = NULL;
					subB = NULL;
					ssd->channel_head[channel].chip_head[chip].die_head[die].token = old_plane;
					return ERROR;
				}
			}
		}
	}

	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeA].free_page<(ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->gc_hard_threshold))
	{
		gc_node = (struct gc_operation *)malloc(sizeof(struct gc_operation));
		alloc_assert(gc_node, "gc_node");
		memset(gc_node, 0, sizeof(struct gc_operation));

		gc_node->next_node = NULL;
		gc_node->chip = chip;
		gc_node->die = die;
		gc_node->plane = planeA;
		gc_node->block = 0xffffffff;
		gc_node->page = 0;
		gc_node->state = GC_WAIT;
		gc_node->priority = GC_UNINTERRUPT;
		gc_node->next_node = ssd->channel_head[channel].gc_command;
		ssd->channel_head[channel].gc_command = gc_node;
		ssd->gc_request++;
	}
	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[planeB].free_page<(ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->gc_hard_threshold))
	{
		gc_node = (struct gc_operation *)malloc(sizeof(struct gc_operation));
		alloc_assert(gc_node, "gc_node");
		memset(gc_node, 0, sizeof(struct gc_operation));

		gc_node->next_node = NULL;
		gc_node->chip = chip;
		gc_node->die = die;
		gc_node->plane = planeB;
		gc_node->block = 0xffffffff;
		gc_node->page = 0;
		gc_node->state = GC_WAIT;
		gc_node->priority = GC_UNINTERRUPT;
		gc_node->next_node = ssd->channel_head[channel].gc_command;
		ssd->channel_head[channel].gc_command = gc_node;
		ssd->gc_request++;
	}

	return SUCCESS;
}




/********************************************
*函数的功能就是让两个位置不同的page位置相同
*********************************************/
struct ssd_info *make_same_level(struct ssd_info *ssd, unsigned int channel, unsigned int chip, unsigned int die, unsigned int plane, unsigned int block, unsigned int aim_page)
{
	int i = 0, step, page;
	struct direct_erase *new_direct_erase, *direct_erase_node;

	page = ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].last_write_page + 1;                  /*需要调整的当前块的可写页号*/
	step = aim_page - page;
	while (i<step)
	{
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[page + i].valid_state = 0;     /*表示某一页失效，同时标记valid和free状态都为0*/
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[page + i].free_state = 0;      /*表示某一页失效，同时标记valid和free状态都为0*/
		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].page_head[page + i].lpn = 0;

		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].invalid_page_num++;

		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].free_page_num--;

		ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].free_page--;

		i++;
	}

	ssd->waste_page_count += step;

	ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].last_write_page = aim_page - 1;

	if (ssd->channel_head[channel].chip_head[chip].die_head[die].plane_head[plane].blk_head[block].invalid_page_num == ssd->parameter->page_block)    /*该block中全是invalid的页，可以直接删除*/
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
*在处理高级命令的写子请求时，这个函数的功能就是计算处理时间以及处理的状态转变
*功能还不是很完善，需要完善，修改时注意要分为静态分配和动态分配两种情况
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
*函数的功能就是把子请求从ssd->subs_w_head或者ssd->channel_head[channel].subs_w_head上删除
******************************************************************************************/
struct ssd_info *delete_from_channel(struct ssd_info *ssd, unsigned int channel, struct sub_request * sub_req)
{
	struct sub_request *sub, *p;

	/******************************************************************
	*完全动态分配子请求就在ssd->subs_w_head上
	*不是完全动态分配子请求就在ssd->channel_head[channel].subs_w_head上
	*******************************************************************/
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

				if (sub == ssd->subs_w_head)                                                     /*将这个子请求从sub request队列中删除*/
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
*函数的功能是在处理读子请求的高级命令时，需要找与one_page相匹配的另外一个page即two_page
*没有找到可以和one_page执行two plane或者interleave操作的页,需要将one_page向后移一个节点
*****************************************************************************************/
struct sub_request *find_interleave_twoplane_page(struct ssd_info *ssd, struct sub_request *one_page, unsigned int command)
{
	struct sub_request *two_page;
	two_page = malloc(sizeof(*two_page));

	if (one_page->lpn == 13926)
	{
		//printf("\n");
	}




	if (one_page->lpn == 21057)
	{
		//printf("\n");
	}

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
						return two_page;                                                       /*找到了与one_page可以执行two plane操作的页*/
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
			if (two_page == NULL)                                                               /*没有找到可以和one_page执行two_plane操作的页,需要将one_page向后移一个节点*/
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
*在处理读子请求高级命令时，利用这个还是查找可以执行高级命令的sub_request
**************************************************************************/
int find_interleave_twoplane_sub_request(struct ssd_info * ssd, unsigned int channel, struct sub_request ** sub_request_one, struct sub_request ** sub_request_two, unsigned int command)
{
	*sub_request_one = ssd->channel_head[channel].subs_r_head;


	while ((*sub_request_one) != NULL)
	{
		(*sub_request_two) = find_interleave_twoplane_page(ssd, *sub_request_one, command);                //*找出两个可以做two_plane或者interleave的read子请求，包括位置条件和时间条件

		if (*sub_request_two == NULL)
		{
			*sub_request_one = (*sub_request_one)->next_node;
		}
		else if (*sub_request_two != NULL)                                                            //*找到了两个可以执行two plane操作的页
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


/**************************************************************************
*这个函数非常重要，读子请求的状态转变，以及时间的计算都通过这个函数来处理
*还有写子请求的执行普通命令时的状态，以及时间的计算也是通过这个函数来处理的
****************************************************************************/
Status go_one_step(struct ssd_info * ssd, struct sub_request * sub1, struct sub_request *sub2, unsigned int aim_state, unsigned int command)
{
	unsigned int i = 0, j = 0, k = 0, m = 0;
	long long time = 0;
	struct sub_request * sub = NULL;
	struct sub_request * sub_twoplane_one = NULL, *sub_twoplane_two = NULL;
	struct sub_request * sub_interleave_one = NULL, *sub_interleave_two = NULL;
	struct local * location = NULL;

	struct buffer_group *update_buffer_node = NULL, key;

	if (sub1 == NULL)
	{
		return ERROR;
	}

	/***************************************************************************************************
	*处理普通命令时，读子请求的目标状态分为以下几种情况SR_R_READ，SR_R_C_A_TRANSFER，SR_R_DATA_TRANSFER
	*写子请求的目标状态只有SR_W_TRANSFER
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
			*这个目标状态是指flash处于读数据的状态，sub的下一状态就应该是传送数据SR_R_DATA_TRANSFER
			*这时与channel无关，只与chip有关所以要修改chip的状态为CHIP_READ_BUSY，下一个状态就是CHIP_DATA_TRANSFER
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
			*目标状态是命令地址传输时，sub的下一个状态就是SR_R_READ
			*这个状态与channel，chip有关，所以要修改channel，chip的状态分别为CHANNEL_C_A_TRANSFER，CHIP_C_A_TRANSFER
			*下一状态分别为CHANNEL_IDLE，CHIP_READ_BUSY
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
			*目标状态是数据传输时，sub的下一个状态就是完成状态SR_COMPLETE
			*这个状态的处理也与channel，chip有关，所以channel，chip的当前状态变为CHANNEL_DATA_TRANSFER，CHIP_DATA_TRANSFER
			*下一个状态分别为CHANNEL_IDLE，CHIP_IDLE。
			***************************************************************************************************************/
			sub->current_time = ssd->current_time;
			sub->current_state = SR_R_DATA_TRANSFER;
			sub->next_state = SR_COMPLETE;
			sub->next_state_predict_time = ssd->current_time + (sub->size*ssd->parameter->subpage_capacity)*ssd->parameter->time_characteristics.tRC;
			sub->complete_time = sub->next_state_predict_time;


			if (sub->update_read_flag == 1)
			{
				sub->update_read_flag = 0;
				//更改buff存的部分写的扇区大小
				key.group = sub->lpn;
				update_buffer_node = (struct buffer_group*)avlTreeFind(ssd->dram->buffer, (TREE_NODE *)&key);    /*在平衡二叉树中寻找buffer node*/
				update_buffer_node->stored = sub->state | update_buffer_node->stored;
				update_buffer_node->dirty_clean = sub->state | update_buffer_node->stored;
				update_buffer_node->page_type = 0;
				buffer_full_flag = 0;   //解除buff的阻塞

			}


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
			*这是处理写子请求时，状态的转变以及时间的计算
			*虽然写子请求的处理状态也像读子请求那么多，但是写请求都是从上往plane中传输数据
			*这样就可以把几个状态当一个状态来处理，就当成SR_W_TRANSFER这个状态来处理，sub的下一个状态就是完成状态了
			*此时channel，chip的当前状态变为CHANNEL_TRANSFER，CHIP_WRITE_BUSY
			*下一个状态变为CHANNEL_IDLE，CHIP_IDLE
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
		*高级命令TWO_PLANE的处理，这里的TWO_PLANE高级命令是读子请求的高级命令
		*状态转变与普通命令一样，不同的是在SR_R_C_A_TRANSFER时计算时间是串行的，因为共用一个通道channel
		*还有SR_R_DATA_TRANSFER也是共用一个通道
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
			{
				sub_twoplane_one->update_read_flag = 0;
				//更改buff存的部分写的扇区大小
				key.group = sub_twoplane_one->lpn;
				update_buffer_node = (struct buffer_group*)avlTreeFind(ssd->dram->buffer, (TREE_NODE *)&key);    /*在平衡二叉树中寻找buffer node*/
				update_buffer_node->stored = sub_twoplane_one->state | update_buffer_node->stored;
				update_buffer_node->dirty_clean = sub_twoplane_one->state | update_buffer_node->stored;
				update_buffer_node->page_type = 0;
				buffer_full_flag = 0;
			}
			else if (sub_twoplane_two->update_read_flag == 1)
			{
				sub_twoplane_two->update_read_flag = 0;
				//更改buff存的部分写的扇区大小
				key.group = sub_twoplane_two->lpn;
				update_buffer_node = (struct buffer_group*)avlTreeFind(ssd->dram->buffer, (TREE_NODE *)&key);    /*在平衡二叉树中寻找buffer node*/
				update_buffer_node->stored = sub_twoplane_two->state | update_buffer_node->stored;
				update_buffer_node->dirty_clean = sub_twoplane_two->state | update_buffer_node->stored;
				update_buffer_node->page_type = 0;
				buffer_full_flag = 0;
			}

			ssd->channel_head[location->channel].current_state = CHANNEL_DATA_TRANSFER;
			ssd->channel_head[location->channel].current_time = ssd->current_time;
			ssd->channel_head[location->channel].next_state = CHANNEL_IDLE;
			ssd->channel_head[location->channel].next_state_predict_time = sub_twoplane_one->next_state_predict_time;

			ssd->channel_head[location->channel].chip_head[location->chip].current_state = CHIP_DATA_TRANSFER;
			ssd->channel_head[location->channel].chip_head[location->chip].current_time = ssd->current_time;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state = CHIP_IDLE;
			ssd->channel_head[location->channel].chip_head[location->chip].next_state_predict_time = sub_twoplane_one->next_state_predict_time;

			//读状态转换完成，此时plane寄存器值置为初始值
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