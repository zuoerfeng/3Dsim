/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName： ssd.c
Author: Zuo Lu 		Version: 1.0	Date:2017/04/06
Description: System main function c file, Contains the basic flow of simulation.
Mainly includes: initialization, make_aged, pre_process_page three parts

History:
<contributor>     <time>        <version>       <desc>                   <e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim       617376665@qq.com

*****************************************************************************************************************************/

#define _CRTDBG_MAP_ALLOC
 
#include <stdlib.h>
#include <crtdbg.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <crtdbg.h>  


#include "ssd.h"
#include "initialize.h"
#include "flash.h"
#include "buffer.h"
#include "interface.h"
#include "ftl.h"
#include "fcl.h"


//Global variable
int make_age_free_page = 0;
int plane_cmplt = 0;
int buffer_full_flag = 0;
__int64 request_lz_count = 0;
int trace_over_flag = 0;
int lz_k=0;
__int64 compare_time = 0;



/********************************************************************************************************************************
1，main函数中initiatio()函数用来初始化ssd,；2，make_aged()函数使SSD成为aged，aged的ssd相当于使用过一段时间的ssd，里面有失效页，
non_aged的ssd是新的ssd，无失效页，失效页的比例可以在初始化参数中设置；3，pre_process_page()函数提前扫一遍读请求，把读请求
的lpn<--->ppn映射关系事先建立好，写请求的lpn<--->ppn映射关系在写的时候再建立，预处理trace防止读请求是读不到数据；4，simulate()是
核心处理函数，trace文件从读进来到处理完成都由这个函数来完成；5，statistic_output()函数将ssd结构中的信息输出到输出文件，输出的是
统计数据和平均数据，输出文件较小，trace_output文件则很大很详细；6，free_all_node()函数释放整个main函数中申请的节点
*********************************************************************************************************************************/

void main()
{
	unsigned  int i,j,k,p,m,n;
	struct ssd_info *ssd;

	#ifdef DEBUG
	printf("enter main\n"); 
	#endif

	ssd=(struct ssd_info*)malloc(sizeof(struct ssd_info));
	alloc_assert(ssd,"ssd");
	memset(ssd,0, sizeof(struct ssd_info));

	ssd=initiation(ssd);

	make_aged(ssd);

	pre_process_page(ssd);

	if (ssd->parameter->aged == 1)
	{
		pre_process_write(ssd);   //将有效块中的free_page全部置为无效，保证最多一个有效块中包含有free page,满足实际ssd的机制
	}

	//此时预处理完成之后，应该保证每个plane的页偏移地址是一致的
	
	for (i=0;i<ssd->parameter->channel_number;i++)
	{
		for (m = 0; m < ssd->parameter->chip_channel[i]; m++)
		{
			for (j = 0; j < ssd->parameter->die_chip; j++)
			{
				for (k = 0; k < ssd->parameter->plane_die; k++)
				{	
					/*
					for (p = 0; p < ssd->parameter->block_plane; p++)
					{
						printf("%d,0,%d,%d,%d,%d:  %5d\n", i, m, j, k, p, ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].last_write_page);
					}
					*/
					printf("%d,0,%d,%d,%d:  %5d\n", i, m, j, k, ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].free_page);
				}
			}
		}
	}

	fprintf(ssd->outputfile,"\t\t\t\t\t\t\t\t\tOUTPUT\n");
	fprintf(ssd->outputfile,"****************** TRACE INFO ******************\n");

	ssd=simulate(ssd);
	statistic_output(ssd);  
	free_all_node(ssd);

	printf("\n");
	printf("the simulation is completed!\n");

	system("pause");
 	_CrtDumpMemoryLeaks();
}


/******************simulate() *********************************************************************
*simulate()是核心处理函数，主要实现的功能包括
*1,从trace文件中获取一条请求，挂到ssd->request
*2，根据ssd是否有dram分别处理读出来的请求，把这些请求处理成为读写子请求，挂到ssd->channel或者ssd上
*3，按照事件的先后来处理这些读写子请求。
*4，输出每条请求的子请求都处理完后的相关信息到outputfile文件中
**************************************************************************************************/
struct ssd_info *simulate(struct ssd_info *ssd)
{
	int flag=1,flag1=0;
	double output_step=0;
	unsigned int a=0,b=0;
	errno_t err;

	
	unsigned int channel_num = 0, chip_num = 0, die_num = 0;
	unsigned int i, j, k,m,p;

	printf("\n");
	printf("begin simulating.......................\n");
	printf("\n");
	printf("\n");
	printf("   ^o^    OK, please wait a moment, and enjoy music and coffee   ^o^    \n");

	//清空预处理时候分配令牌
	channel_num = ssd->parameter->channel_number;
	chip_num = ssd->parameter->chip_channel[0];
	die_num = ssd->parameter->die_chip;
	ssd->token = 0;
	for (i = 0; i < channel_num; i++)
	{
		for (j = 0; j < chip_num; j++)
		{
			for (k = 0; k < die_num; k++)
			{
				ssd->channel_head[i].chip_head[j].die_head[k].token = 0;
			}
			ssd->channel_head[i].chip_head[j].token = 0;
		}
		ssd->channel_head[i].token = 0;
	}


	if((err=fopen_s(&(ssd->tracefile),ssd->tracefilename,"r"))!=0)
	{  
		printf("the trace file can't open\n");
		return NULL;
	}

	fprintf(ssd->outputfile,"      arrive           lsn     size ope     begin time    response time    process time\n");	
	fflush(ssd->outputfile);

	while(flag!=100)      
	{        
		/*interface层*/
		flag = get_requests(ssd);        
		
		/*buffer层*/
		if (flag == 1 || (flag == 0 && ssd->request_work != NULL))
		{   
			//printf("once\n");
			if (ssd->parameter->dram_capacity!=0)
			{
				if (buffer_full_flag == 0)				//buff未阻塞状态方可执行buff操作
				{
					buffer_management(ssd);
					distribute(ssd);
				}
			} 
			else
			{
				no_buffer_distribute(ssd);
			}

			if (ssd->request_work->cmplt_flag == 1)
			{
				if (ssd->request_work != ssd->request_tail)
					ssd->request_work = ssd->request_work->next_node;
				else
					ssd->request_work = NULL;
			}
		}
		

		/*ftl+fcl+flash层*/
		process(ssd);    
		trace_output(ssd);
		
		/*
		if (trace_over_flag == 1)
			flag = 0;
		*/

		if (flag == 0 && ssd->request_queue == NULL)
			flag = 100;
	}

	fclose(ssd->tracefile);
	return ssd;
}


/********************************************************
*这个函数的主要功能是主控读子请求和写子请求的状态变化处理
*********************************************************/

struct ssd_info *process(struct ssd_info *ssd)
{

	/*********************************************************************************************************
	*flag_die表示是否因为die的busy，阻塞了时间前进，-1表示没有，非-1表示有阻塞，
	*flag_die的值表示die号,old ppn记录在copyback之前的物理页号，用于判断copyback是否遵守了奇偶地址的限制；
	*two_plane_bit[8],two_plane_place[8]数组成员表示同一个channel上每个die的请求分配情况；
	*chg_cur_time_flag作为是否需要调整当前时间的标志位，当因为channel处于busy导致请求阻塞时，需要调整当前时间；
	*初始认为需要调整，置为1，当任何一个channel处理了传送命令或者数据时，这个值置为0，表示不需要调整；
	**********************************************************************************************************/
	int old_ppn = -1, flag_die = -1;
	unsigned int i,j,k, chan, random_num;
	unsigned int flag = 0, new_write = 0, chg_cur_time_flag = 1, flag2 = 0, flag_gc = 0;
	__int64 time, channel_time = 0x7fffffffffffffff;
	struct sub_request *sub;
	
	unsigned int  m, p;
	unsigned int channel = 0, chip = 0, die = 0;

#ifdef DEBUG
	printf("enter process,  current time:%I64u\n", ssd->current_time);
#endif


	/*********************************************************
	*判断是否有读写子请求，如果有那么flag令为0，没有flag就为1
	*当flag为1时，若ssd中有gc操作这时就可以执行gc操作
	**********************************************************/

	/*ftl层*/
	//主动gc，由于遍历了所有的channel，所有gc操作无效块的擦拭偏移一样，再次写的时候plane的偏移地址都是一样的
	for (i = 0; i<ssd->parameter->channel_number; i++)
	{
		if ((ssd->channel_head[i].subs_r_head == NULL) && (ssd->channel_head[i].subs_w_head == NULL) && (ssd->subs_w_head == NULL))
		{
			flag = 1;
		}
		else
		{
			flag = 0;
			break;
		}
	}


	if (flag == 1)
	{
		ssd->flag = 1;
		if (ssd->gc_request>0)                                                            /*SSD中有gc操作的请求*/
		{
			gc(ssd, 0, 1);                                                                  /*这个gc要求所有channel都必须遍历到*/
		}
		return ssd;
	}
	else
	{
		ssd->flag = 0;
	}

	/*fcl+flash层*/
	time = ssd->current_time;
	services_2_r_cmd_trans_and_complete(ssd);                                            /*处理当前状态是SR_R_C_A_TRANSFER或者当前状态是SR_COMPLETE，或者下一状态是SR_COMPLETE并且下一状态预计时间小于当前状态时间*/
	
	/*****************************************
	*循环处理所有channel上的读写子请求
	*发读请求命令，传读写数据，都需要占用总线，
	******************************************/
	random_num = ssd->program_count%ssd->parameter->channel_number;                        /*产生一个随机数，保证每次从不同的channel开始查询*/
	for (chan = 0; chan<ssd->parameter->channel_number; chan++)
	{
		i = (random_num + chan) % ssd->parameter->channel_number;
		flag = 0;
		flag_gc = 0;																		/*每次进入channel时，将gc的标志位置为0，默认认为没有进行gc操作*/

		//读操作状态推进，要随机遍历所有channel推进状态
		if ((ssd->channel_head[i].current_state == CHANNEL_IDLE) || (ssd->channel_head[i].next_state == CHANNEL_IDLE&&ssd->channel_head[i].next_state_predict_time <= ssd->current_time))
		{
			
			//*ftl层
			if (ssd->gc_request>0)                                                       //有gc操作，需要进行一定的判断
			{
				if (ssd->channel_head[i].gc_command != NULL)
				{
					flag_gc = gc(ssd, i, 0);                                                 //gc函数返回一个值，表示是否执行了gc操作，如果执行了gc操作，这个channel在这个时刻不能服务其他的请求
				}
				if (flag_gc == 1)                                                          //执行过gc操作，需要跳出此次循环
				{
					continue;
				}
			}
			

			/*fcl+flash层*/
			sub = ssd->channel_head[i].subs_r_head;                                        /*先处理读请求*/

			services_2_r_wait(ssd, i, &flag, &chg_cur_time_flag);                           /*处理处于等待状态的读子请求*/

			if ((flag == 0) && (ssd->channel_head[i].subs_r_head != NULL))                      /*if there are no new read request and data is ready in some dies, send these data to controller and response this request*/
			{
				services_2_r_data_trans(ssd, i, &flag, &chg_cur_time_flag);

			}


			//开始进行写请求的状态转变
			if (flag == 0)                                                                  /*if there are no read request to take channel, we can serve write requests*/
			{			
				if (ssd->parameter->dynamic_allocation_priority == 1)
				{
					//这个函数返回代表此时一个channel上的superpage 写完成，进行下一个channel 
					channel = ssd->token;
					chip = ssd->channel_head[channel].token;
					die = ssd->channel_head[channel].chip_head[chip].token;

					//表示当前写过程完成
					services_2_write(ssd, channel, &flag, &chg_cur_time_flag);

					//当多个plane同时写完之后，才能进行channel增加
					if (plane_cmplt == 1)
					{
						ssd->token = (ssd->token + 1) % ssd->parameter->channel_number;
						plane_cmplt = 0;
						if (channel == (ssd->parameter->channel_number - 1))
							ssd->channel_head[ssd->token].chip_head[chip].token = (die + 1) % ssd->parameter->die_chip;
						else
							ssd->channel_head[ssd->token].chip_head[chip].token = die;
					}
					//printf("aaa\n");
					
				}
				else
				{
					services_2_write(ssd, i, &flag, &chg_cur_time_flag); 
				}
				
			}
		}

		/*此时用来查看plane内的偏移地址是否相同，从而验证我们代码的有效性*/
		/*
		for (j = 0; j < ssd->parameter->die_chip; j++)
		{
			for (i = 0; i<ssd->parameter->channel_number; i++)
			{
				for (m = 0; m < ssd->parameter->chip_channel[i]; m++)
				{
					for (k = 0; k < ssd->parameter->plane_die; k++)
					{
						for (p = 0; p < ssd->parameter->block_plane; p++)
						{
							if ((ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num > 0) && (ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num < ssd->parameter->page_block))
							{
								printf("%d %d %d %d %d,%5d,%5d\n", i, m, j, k, p, ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].last_write_page, plane_cmplt);
								//getchar();
							}
						}
					}
				}
			}
		}*/

	}
	return ssd;
}



/**********************************************************************
*trace_output()函数是在每一条请求的所有子请求经过process()函数处理完后，
*打印输出相关的运行结果到outputfile文件中，这里的结果主要是运行的时间
**********************************************************************/
void trace_output(struct ssd_info* ssd){
	int flag = 1;	
	__int64 start_time, end_time;
	struct request *req, *pre_node;
	struct sub_request *sub, *tmp;
	unsigned int i;

#ifdef DEBUG
	printf("enter trace_output,  current time:%I64u\n",ssd->current_time);
#endif

	pre_node=NULL;
	req = ssd->request_queue;
	start_time = 0;
	end_time = 0;

	if(req == NULL)
		return;

	while(req != NULL)	
	{
		sub = req->subs;
		flag = 1;
		start_time = 0;
		end_time = 0;
		if (req->response_time != 0 && req->cmplt_flag == 1)
		{
			fprintf(ssd->outputfile, "%16lld %10d %6d %2d %16lld %16lld %10lld\n", req->time, req->lsn, req->size, req->operation, req->begin_time, req->response_time, req->response_time - req->time);
			fflush(ssd->outputfile);

			if (req->response_time - req->begin_time == 0)
			{
				printf("the response time is 0?? \n");
				getchar();
			}

			if (req->operation == READ)
			{
				ssd->read_request_count++;
				ssd->read_avg = ssd->read_avg + (req->response_time - req->time);
			}
			else
			{
				ssd->write_request_count++;
				ssd->write_avg = ssd->write_avg + (req->response_time - req->time);
			}

			if (pre_node == NULL)
			{
				if (req->next_node == NULL)
				{
					free(req->need_distr_flag);
					req->need_distr_flag = NULL;
					free(req);
					req = NULL;
					ssd->request_queue = NULL;
					ssd->request_tail = NULL;
					ssd->request_queue_length--;
				}
				else
				{
					ssd->request_queue = req->next_node;
					pre_node = req;
					req = req->next_node;
					free(pre_node->need_distr_flag);
					pre_node->need_distr_flag = NULL;
					free((void *)pre_node);
					pre_node = NULL;
					ssd->request_queue_length--;
				}
			}
			else
			{
				if (req->next_node == NULL)
				{
					pre_node->next_node = NULL;
					free(req->need_distr_flag);
					req->need_distr_flag = NULL;
					free(req);
					req = NULL;
					ssd->request_tail = pre_node;
					ssd->request_queue_length--;
				}
				else
				{
					pre_node->next_node = req->next_node;
					free(req->need_distr_flag);
					req->need_distr_flag = NULL;
					free((void *)req);
					req = pre_node->next_node;
					ssd->request_queue_length--;
				}
			}
		}
		else if (req->response_time == 0 && req->cmplt_flag == 1)
		{
			flag = 1;
			while (sub != NULL)
			{
				if ( (sub->lpn == 13992 | sub->lpn == 13991)  &&  (req->lsn == 91389))
					printf("lz\n");

				if (req->lsn == 91389)
					printf("lz\n");

				if (start_time == 0)
					start_time = sub->begin_time;
				if (start_time > sub->begin_time)
					start_time = sub->begin_time;
				if (end_time < sub->complete_time)
					end_time = sub->complete_time;

				/*
				if (trace_over_flag == 1)
				{
					compare_time = ssd->current_time * 10;

				}
				else
					compare_time = ssd->current_time;
				*/

				if ((sub->current_state == SR_COMPLETE) || ((sub->next_state == SR_COMPLETE) && (sub->next_state_predict_time <= ssd->current_time)))	// if any sub-request is not completed, the request is not completed
				{
					sub = sub->next_subs;
					if (end_time - start_time == 0)
					{
						printf("the response time is 0?? \n");
						getchar();
					}
				}
				else
				{
					flag = 0;
					break;
				}

			}

			if (flag == 1)
			{
				//fprintf(ssd->outputfile,"%10I64u %10u %6u %2u %16I64u %16I64u %10I64u\n",req->time,req->lsn, req->size, req->operation, start_time, end_time, end_time-req->time);
				fprintf(ssd->outputfile, "%16lld %10d %6d %2d %16lld %16lld %10lld\n", req->time, req->lsn, req->size, req->operation, start_time, end_time, end_time - req->time);
				fflush(ssd->outputfile);

				if (end_time - start_time == 0)
				{
					printf("the response time is 0?? \n");
					getchar();
				}

				if (req->operation == READ)
				{
					ssd->read_request_count++;
					ssd->read_avg = ssd->read_avg + (end_time - req->time);
				}
				else
				{
					ssd->write_request_count++;
					ssd->write_avg = ssd->write_avg + (end_time - req->time);
				}


				if (req->lsn == 91389 && req->size == 64)
				{
					printf("ERROR\n");
				}


				//该请求执行完成，释放所有子请求
				while (req->subs != NULL)
				{
					tmp = req->subs;
					req->subs = tmp->next_subs;
					if (tmp->update != NULL)
					{
						free(tmp->update->location);
						tmp->update->location = NULL;
						free(tmp->update);
						tmp->update = NULL;
					}
					free(tmp->location);
					tmp->location = NULL;
					free(tmp);
					tmp = NULL;
				}

				if (pre_node == NULL)
				{
					if (req->next_node == NULL)
					{
						free(req->need_distr_flag);
						req->need_distr_flag = NULL;
						free(req);
						req = NULL;
						ssd->request_queue = NULL;
						ssd->request_tail = NULL;
						ssd->request_queue_length--;
					}
					else
					{
						ssd->request_queue = req->next_node;
						pre_node = req;
						req = req->next_node;
						free(pre_node->need_distr_flag);
						pre_node->need_distr_flag = NULL;
						free(pre_node);
						pre_node = NULL;
						ssd->request_queue_length--;
					}
				}
				else
				{
					if (req->next_node == NULL)
					{
						pre_node->next_node = NULL;
						free(req->need_distr_flag);
						req->need_distr_flag = NULL;
						free(req);
						req = NULL;
						ssd->request_tail = pre_node;
						ssd->request_queue_length--;
					}
					else
					{
						pre_node->next_node = req->next_node;
						free(req->need_distr_flag);
						req->need_distr_flag = NULL;
						free(req);
						req = pre_node->next_node;
						ssd->request_queue_length--;
					}

				}
			}
			else
			{
				pre_node = req;
				req = req->next_node;
			}
		}
		else
		{
			pre_node = req;
			req = req->next_node;
		}
	}
}


/*******************************************************************************
*statistic_output()函数主要是输出处理完一条请求后的相关处理信息。
*1，计算出每个plane的擦除次数即plane_erase和总的擦除次数即erase
*2，打印min_lsn，max_lsn，read_count，program_count等统计信息到文件outputfile中。
*3，打印相同的信息到文件statisticfile中
*******************************************************************************/
void statistic_output(struct ssd_info *ssd)
{
	unsigned int lpn_count=0,i,j,k,m,p,erase=0,plane_erase=0;
	unsigned int blk_read = 0, plane_read = 0;
	unsigned int blk_write = 0, plane_write = 0;
	unsigned int pre_plane_write = 0;
	double gc_energy=0.0;
#ifdef DEBUG
	printf("enter statistic_output,  current time:%I64u\n",ssd->current_time);
#endif

	for(i=0;i<ssd->parameter->channel_number;i++)
	{
		for (p = 0; p < ssd->parameter->chip_channel[i]; p++)
		{
			for (j = 0; j < ssd->parameter->die_chip; j++)
			{
				for (k = 0; k < ssd->parameter->plane_die; k++)
				{
					for (m = 0; m < ssd->parameter->block_plane; m++)
					{
						if (ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].blk_head[m].erase_count > 0)
						{
							ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_erase_count += ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].blk_head[m].erase_count;
						}

						if (ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].blk_head[m].page_read_count > 0)
						{
							ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_read_count += ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].blk_head[m].page_read_count;
						}

						if (ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].blk_head[m].page_write_count > 0)
						{
							ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_program_count += ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].blk_head[m].page_write_count;
						}

						if (ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].blk_head[m].pre_write_count > 0)
						{
							ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].pre_plane_write_count += ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].blk_head[m].pre_write_count;
						}
					}
					fprintf(ssd->outputfile, "the %d channel, %d chip, %d die, %d plane has : ", i, p, j, k);
					fprintf(ssd->outputfile, "%3d erase operations,", ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_erase_count);
					fprintf(ssd->outputfile, "%3d read operations,", ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_read_count);
					fprintf(ssd->outputfile, "%3d write operations,", ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_program_count);
					fprintf(ssd->outputfile, "%3d pre_process write operations\n", ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].pre_plane_write_count);
					
					fprintf(ssd->statisticfile, "the %d channel, %d chip, %d die, %d plane has : ", i, p, j, k);
					fprintf(ssd->statisticfile, "%3d erase operations,", ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_erase_count);
					fprintf(ssd->statisticfile, "%3d read operations,", ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_read_count);
					fprintf(ssd->statisticfile, "%3d write operations,", ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].plane_program_count);
					fprintf(ssd->statisticfile, "%3d pre_process write operations\n", ssd->channel_head[i].chip_head[p].die_head[j].plane_head[k].pre_plane_write_count);
				}
			}
		}
	}

	fprintf(ssd->outputfile,"\n");
	fprintf(ssd->outputfile,"\n");
	fprintf(ssd->outputfile,"---------------------------statistic data---------------------------\n");	 
	fprintf(ssd->outputfile,"min lsn: %13d\n",ssd->min_lsn);	
	fprintf(ssd->outputfile,"max lsn: %13d\n",ssd->max_lsn);
	fprintf(ssd->outputfile,"read count: %13d\n",ssd->read_count);	  
	fprintf(ssd->outputfile,"the read operation leaded by un-covered update count: %13d\n",ssd->update_read_count);
	fprintf(ssd->outputfile, "the read operation leaded by gc read count: %13d\n", ssd->gc_read_count);
	fprintf(ssd->outputfile, "\n");
	fprintf(ssd->outputfile, "program count: %13d\n", ssd->program_count);
	fprintf(ssd->outputfile, "the write operation leaded by pre_process write count: %13d\n", ssd->pre_all_write);
	fprintf(ssd->outputfile, "the write operation leaded by un-covered update count: %13d\n", ssd->update_write_count);
	fprintf(ssd->outputfile, "the write operation leaded by gc read count: %13d\n", ssd->gc_write_count);
	fprintf(ssd->outputfile, "\n");
	fprintf(ssd->outputfile,"erase count: %13d\n",ssd->erase_count);
	fprintf(ssd->outputfile,"direct erase count: %13d\n",ssd->direct_erase_count);
	//fprintf(ssd->outputfile,"copy back count: %13d\n",ssd->copy_back_count);
	fprintf(ssd->outputfile,"multi-plane program count: %13d\n",ssd->m_plane_prog_count);
	fprintf(ssd->outputfile,"multi-plane read count: %13d\n",ssd->m_plane_read_count);
	//fprintf(ssd->outputfile,"interleave write count: %13d\n",ssd->interleave_count);
	//fprintf(ssd->outputfile,"interleave read count: %13d\n",ssd->interleave_read_count);
	//fprintf(ssd->outputfile,"interleave two plane and one program count: %13d\n",ssd->inter_mplane_prog_count);
	//fprintf(ssd->outputfile,"interleave two plane count: %13d\n",ssd->inter_mplane_count);
	//fprintf(ssd->outputfile,"gc copy back count: %13d\n",ssd->gc_copy_back);
	fprintf(ssd->outputfile,"write flash count: %13d\n",ssd->write_flash_count);
	fprintf(ssd->outputfile, "\n");
	//fprintf(ssd->outputfile,"interleave erase count: %13d\n",ssd->interleave_erase_count);
	//fprintf(ssd->outputfile,"multiple plane erase count: %13d\n",ssd->mplane_erase_conut);
	//fprintf(ssd->outputfile,"interleave multiple plane erase count: %13d\n",ssd->interleave_mplane_erase_count);
	fprintf(ssd->outputfile,"read request count: %13d\n",ssd->read_request_count);
	fprintf(ssd->outputfile,"write request count: %13d\n",ssd->write_request_count);
	fprintf(ssd->outputfile, "\n");
	fprintf(ssd->outputfile,"read request average size: %13f\n",ssd->ave_read_size);
	fprintf(ssd->outputfile,"write request average size: %13f\n",ssd->ave_write_size);
	fprintf(ssd->outputfile, "\n");
//	fprintf(ssd->outputfile,"read request average response time: %16I64u\n",ssd->read_avg/ssd->read_request_count);
	fprintf(ssd->outputfile,"write request average response time: %16I64u\n",ssd->write_avg/ssd->write_request_count);
	fprintf(ssd->outputfile, "\n");
	fprintf(ssd->outputfile,"buffer read hits: %13d\n",ssd->dram->buffer->read_hit);
	fprintf(ssd->outputfile,"buffer read miss: %13d\n",ssd->dram->buffer->read_miss_hit);
	fprintf(ssd->outputfile,"buffer write hits: %13d\n",ssd->dram->buffer->write_hit);
	fprintf(ssd->outputfile,"buffer write miss: %13d\n",ssd->dram->buffer->write_miss_hit);
	fprintf(ssd->outputfile, "\n");
	fflush(ssd->outputfile);

	fclose(ssd->outputfile);


	fprintf(ssd->statisticfile, "\n");
	fprintf(ssd->statisticfile, "\n");
	fprintf(ssd->statisticfile, "---------------------------statistic data---------------------------\n");
	fprintf(ssd->statisticfile, "min lsn: %13d\n", ssd->min_lsn);
	fprintf(ssd->statisticfile, "max lsn: %13d\n", ssd->max_lsn);
	fprintf(ssd->statisticfile, "read count: %13d\n", ssd->read_count);
	fprintf(ssd->statisticfile, "the read operation leaded by un-covered update count: %13d\n", ssd->update_read_count);
	fprintf(ssd->statisticfile, "the read operation leaded by gc read count: %13d\n", ssd->gc_read_count);
	fprintf(ssd->statisticfile, "\n");
	fprintf(ssd->statisticfile, "program count: %13d\n", ssd->program_count);
	fprintf(ssd->statisticfile, "the write operation leaded by pre_process write count: %13d\n", ssd->pre_all_write);
	fprintf(ssd->statisticfile, "the write operation leaded by un-covered update count: %13d\n", ssd->update_write_count);
	fprintf(ssd->statisticfile, "the write operation leaded by gc read count: %13d\n", ssd->gc_write_count);
	fprintf(ssd->statisticfile, "\n");
	fprintf(ssd->statisticfile,"erase count: %13d\n",ssd->erase_count);	  
	fprintf(ssd->statisticfile,"direct erase count: %13d\n",ssd->direct_erase_count);
	fprintf(ssd->statisticfile, "\n");
	//fprintf(ssd->statisticfile,"copy back count: %13d\n",ssd->copy_back_count);
	fprintf(ssd->statisticfile,"multi-plane program count: %13d\n",ssd->m_plane_prog_count);
	fprintf(ssd->statisticfile,"multi-plane read count: %13d\n",ssd->m_plane_read_count);
	fprintf(ssd->statisticfile, "\n");
	//fprintf(ssd->statisticfile,"interleave count: %13d\n",ssd->interleave_count);
	//fprintf(ssd->statisticfile,"interleave read count: %13d\n",ssd->interleave_read_count);
	//fprintf(ssd->statisticfile,"interleave two plane and one program count: %13d\n",ssd->inter_mplane_prog_count);
	//fprintf(ssd->statisticfile,"interleave two plane count: %13d\n",ssd->inter_mplane_count);
	//fprintf(ssd->statisticfile,"gc copy back count: %13d\n",ssd->gc_copy_back);
	fprintf(ssd->statisticfile,"write flash count: %13d\n",ssd->write_flash_count);
	fprintf(ssd->statisticfile, "\n");
	//fprintf(ssd->statisticfile,"waste page count: %13d\n",ssd->waste_page_count);
	//fprintf(ssd->statisticfile,"interleave erase count: %13d\n",ssd->interleave_erase_count);
	//fprintf(ssd->statisticfile,"multiple plane erase count: %13d\n",ssd->mplane_erase_conut);
	//fprintf(ssd->statisticfile,"interleave multiple plane erase count: %13d\n",ssd->interleave_mplane_erase_count);
	fprintf(ssd->statisticfile,"read request count: %13d\n",ssd->read_request_count);
	fprintf(ssd->statisticfile, "write request count: %13d\n", ssd->write_request_count);
	fprintf(ssd->statisticfile, "\n");
	fprintf(ssd->statisticfile,"read request average size: %13f\n",ssd->ave_read_size);
	fprintf(ssd->statisticfile,"write request average size: %13f\n",ssd->ave_write_size);
	fprintf(ssd->statisticfile, "\n");
//	fprintf(ssd->statisticfile,"read request average response time: %16I64u\n",ssd->read_avg/ssd->read_request_count);
	fprintf(ssd->statisticfile,"write request average response time: %16I64u\n",ssd->write_avg/ssd->write_request_count);
	fprintf(ssd->statisticfile, "\n");
	fprintf(ssd->statisticfile,"buffer read hits: %13d\n",ssd->dram->buffer->read_hit);
	fprintf(ssd->statisticfile,"buffer read miss: %13d\n",ssd->dram->buffer->read_miss_hit);
	fprintf(ssd->statisticfile,"buffer write hits: %13d\n",ssd->dram->buffer->write_hit);
	fprintf(ssd->statisticfile,"buffer write miss: %13d\n",ssd->dram->buffer->write_miss_hit);

//	fprintf(ssd->statisticfile, "buffer write hit request count : %13d\n", request_lz_count);

	fprintf(ssd->statisticfile, "\n");
	fflush(ssd->statisticfile);

	fclose(ssd->statisticfile);
}




/***********************************************
*free_all_node()函数的作用就是释放所有申请的节点
************************************************/
void free_all_node(struct ssd_info *ssd)
{
	unsigned int i,j,k,l,n;
	struct buffer_group *pt=NULL;
	struct direct_erase * erase_node=NULL;
	for (i=0;i<ssd->parameter->channel_number;i++)
	{
		for (j=0;j<ssd->parameter->chip_channel[0];j++)
		{
			for (k=0;k<ssd->parameter->die_chip;k++)
			{
				for (l=0;l<ssd->parameter->plane_die;l++)
				{
					for (n=0;n<ssd->parameter->block_plane;n++)
					{
						free(ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[n].page_head);
						ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[n].page_head=NULL;
					}
					free(ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head);
					ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head=NULL;
					while(ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].erase_node!=NULL)
					{
						erase_node=ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].erase_node;
						ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].erase_node=erase_node->next_node;
						free(erase_node);
						erase_node=NULL;
					}
				}
				
				free(ssd->channel_head[i].chip_head[j].die_head[k].plane_head);
				ssd->channel_head[i].chip_head[j].die_head[k].plane_head=NULL;
			}
			free(ssd->channel_head[i].chip_head[j].die_head);
			ssd->channel_head[i].chip_head[j].die_head=NULL;
		}
		free(ssd->channel_head[i].chip_head);
		ssd->channel_head[i].chip_head=NULL;
	}
	free(ssd->channel_head);
	ssd->channel_head=NULL;

	avlTreeDestroy( ssd->dram->buffer);
	ssd->dram->buffer=NULL;
	
	free(ssd->dram->map->map_entry);
	ssd->dram->map->map_entry=NULL;
	free(ssd->dram->map);
	ssd->dram->map=NULL;
	free(ssd->dram);
	ssd->dram=NULL;
	free(ssd->parameter);
	ssd->parameter=NULL;

	free(ssd);
	ssd=NULL;
}


/*****************************************************************************
*make_aged()函数的作用就死模拟真实的用过一段时间的ssd，
*那么这个ssd的相应的参数就要改变，所以这个函数实质上就是对ssd中各个参数的赋值。
******************************************************************************/
struct ssd_info *make_aged(struct ssd_info *ssd)
{
	unsigned int i,j,k,l,m,n,ppn;
	int threshould,flag=0;
    
	if (ssd->parameter->aged==1)
	{
		//threshold表示一个plane中有多少页需要提前置为失效
		threshould=(int)(ssd->parameter->block_plane*ssd->parameter->page_block*ssd->parameter->aged_ratio);  
		for (i=0;i<ssd->parameter->channel_number;i++)
			for (j=0;j<ssd->parameter->chip_channel[i];j++)
				for (k=0;k<ssd->parameter->die_chip;k++)
					for (l=0;l<ssd->parameter->plane_die;l++)
					{  
						flag=0;
						for (m=0;m<ssd->parameter->block_plane;m++)
						{  
							if (flag>=threshould)
							{
								break;
							}
							//注意这里旧化率+1，表示最后满足了旧化条件还会剩余一点全部空闲的block.
							for (n=0;n<(ssd->parameter->page_block*ssd->parameter->aged_ratio+1);n++)
							{  
								ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[m].page_head[n].valid_state=0;        //表示某一页失效，同时标记valid和free状态都为0
								ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[m].page_head[n].free_state=0;         //表示某一页失效，同时标记valid和free状态都为0
								ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[m].page_head[n].lpn=0;  //把valid_state free_state lpn都置为0表示页失效，检测的时候三项都检测，单独lpn=0可以是有效页
								ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[m].free_page_num--;
								ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[m].invalid_page_num++;
								ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[m].last_write_page++;
								ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].free_page--;
								flag++;

								ppn=find_ppn(ssd,i,j,k,l,m,n);
							
							}
							make_age_free_page = ssd->channel_head[i].chip_head[j].die_head[k].plane_head[l].blk_head[m].free_page_num;
						} 
					}	 
	}  
	else
	{
		return ssd;
	}

	return ssd;
}



/*********************************************************************************************
*将有效块中的free_page全部置为无效，保证最多一个有效块中包含有free page,满足实际ssd的机制
*遍历所有的块，当有效块中包含有旧化遗留的free_page的时候，将所有的free_page置无效，并将无
*效块放入gc链。
*********************************************************************************************/
struct ssd_info *pre_process_write(struct ssd_info *ssd)
{
	unsigned  int i, j, k, p, m, n;
	struct direct_erase *direct_erase_node, *new_direct_erase;

	for (i = 0; i<ssd->parameter->channel_number; i++)
	{
		for (m = 0; m < ssd->parameter->chip_channel[i]; m++)
		{
			for (j = 0; j < ssd->parameter->die_chip; j++)
			{
				for (k = 0; k < ssd->parameter->plane_die; k++)
				{
					//查看是否有空闲块
					for (p = 0; p < ssd->parameter->block_plane; p++)
					{
						//if ((ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num == make_age_free_page) && (ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num < ssd->parameter->page_block))
						if ((ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num > 0) && (ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num < ssd->parameter->page_block))
						{
							if (ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num == make_age_free_page)
							{
								ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].free_page = ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].free_page - ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num;
								ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num = 0;
								ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].invalid_page_num = ssd->parameter->page_block;
								ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].last_write_page = ssd->parameter->page_block - 1;

								for (n = 0; n < ssd->parameter->page_block; n++)
								{
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].page_head[n].valid_state = 0;        //表示某一页失效，同时标记valid和free状态都为0
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].page_head[n].free_state = 0;         //表示某一页失效，同时标记valid和free状态都为0
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].page_head[n].lpn = 0;  //把valid_state free_state lpn都置为0表示页失效，检测的时候三项都检测，单独lpn=0可以是有效页
								}
								//整个页是无效页，故要将此block无效块 并添加到gc链上

								new_direct_erase = (struct direct_erase *)malloc(sizeof(struct direct_erase));
								alloc_assert(new_direct_erase, "new_direct_erase");
								memset(new_direct_erase, 0, sizeof(struct direct_erase));

								new_direct_erase->block = p;  //给出当前无效块的块号
								new_direct_erase->next_node = NULL;
								direct_erase_node = ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].erase_node;
								if (direct_erase_node == NULL)
								{
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].erase_node = new_direct_erase;
								}
								else
								{
									new_direct_erase->next_node = ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].erase_node;
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].erase_node = new_direct_erase;
								}
							}
							else
							{
								for (n = (ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].last_write_page + 1); n < ssd->parameter->page_block; n++)
								{
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].page_head[n].valid_state = 0;        //表示某一页失效，同时标记valid和free状态都为0
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].page_head[n].free_state = 0;         //表示某一页失效，同时标记valid和free状态都为0
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].page_head[n].lpn = 0;  //把valid_state free_state lpn都置为0表示页失效，检测的时候三项都检测，单独lpn=0可以是有效页


									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].free_page = ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].free_page - ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num;
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num = 0;
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].invalid_page_num = ssd->parameter->page_block;
									ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].last_write_page = ssd->parameter->page_block - 1;


								}
							}
						}
						//printf("%d,0,%d,%d,%d:%5d\n", i, j, k, p, ssd->channel_head[i].chip_head[m].die_head[j].plane_head[k].blk_head[p].free_page_num);
					}
				}
			}
		}
	}
return ssd;
}


/************************************************
*断言,当打开文件失败时，输出“open 文件名 error”
*************************************************/
void file_assert(int error, char *s)
{
	if (error == 0) return;
	printf("open %s error\n", s);
	getchar();
	exit(-1);
}

/*****************************************************
*断言,当申请内存空间失败时，输出“malloc 变量名 error”
******************************************************/
void alloc_assert(void *p, char *s)//断言
{
	if (p != NULL) return;
	printf("malloc %s error\n", s);
	getchar();
	exit(-1);
}

/*********************************************************************************
*断言
*A，读到的time_t，device，lsn，size，ope都<0时，输出“trace error:.....”
*B，读到的time_t，device，lsn，size，ope都=0时，输出“probable read a blank line”
**********************************************************************************/
void trace_assert(_int64 time_t, int device, unsigned int lsn, int size, int ope)//断言
{
	if (time_t <0 || device < 0 || lsn < 0 || size < 0 || ope < 0)
	{
		printf("trace error:%I64u %d %d %d %d\n", time_t, device, lsn, size, ope);
		getchar();
		exit(-1);
	}
	if (time_t == 0 && device == 0 && lsn == 0 && size == 0 && ope == 0)
	{
		printf("probable read a blank line\n");
		getchar();
	}
}