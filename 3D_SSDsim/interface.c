/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName： ssd.c
Author: Zuo Lu 		Version: 1.0	Date:2017/04/06
Description: 
Interface layer: to complete the IO request to obtain, and converted into the corresponding page-level SSD request

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

#include "ssd.h"
#include "initialize.h"
#include "flash.h"
#include "buffer.h"
#include "interface.h"
#include "ftl.h"
#include "fcl.h"

extern int buffer_full_flag ;
extern int trace_over_flag ;


/********    get_request    ******************************************************
*	1.get requests that arrived already
*	2.add those request node to ssd->reuqest_queue
*	return	0: reach the end of the trace
*			-1: no request has been added
*			1: add one request to list
*SSD模拟器有三种驱动方式:时钟驱动(精确，太慢) 事件驱动(本程序采用) trace驱动()，
*两种方式推进事件：channel/chip状态改变、trace文件请求达到。
*channel/chip状态改变和trace文件请求到达是散布在时间轴上的点，每次从当前状态到达
*下一个状态都要到达最近的一个状态，每到达一个点执行一次process
********************************************************************************/
int get_requests(struct ssd_info *ssd)
{
	char buffer[200];
	unsigned int lsn = 0;
	int device, size, ope, large_lsn, i = 0, j = 0;
	struct request *request1;
	int flag = 1;
	long filepoint;
	__int64 time_t;
	__int64 nearest_event_time;

	extern __int64 request_lz_count;

#ifdef DEBUG
	printf("enter get_requests,  current time:%I64u\n", ssd->current_time);
#endif
	
	if (trace_over_flag == 1)
		return 0;

	/*
	filepoint = ftell(ssd->tracefile);
	fgets(buffer, 200, ssd->tracefile);
	sscanf(buffer, "%I64u %d %d %d %d", &time_t, &device, &lsn, &size, &ope);
	*/

	while (TRUE)
	{
		filepoint = ftell(ssd->tracefile);
		fgets(buffer, 200, ssd->tracefile);
		sscanf(buffer, "%I64u %d %d %d %d", &time_t, &device, &lsn, &size, &ope);

		if (size < (ssd->parameter->dram_capacity / 512))
			break;

		if (feof(ssd->tracefile))      //判断是否读完整个trace,读完则跳出整个循环
			break;
	}

	if ((device<0) && (lsn<0) && (size<0) && (ope<0))
	{
		return 100;
	}
	if (lsn<ssd->min_lsn)
		ssd->min_lsn = lsn;
	if (lsn>ssd->max_lsn)
		ssd->max_lsn = lsn;

	/******************************************************************************************************
	*上层文件系统发送给SSD的任何读写命令包括两个部分（LSN，size） LSN是逻辑扇区号，对于文件系统而言，它所看到的存
	*储空间是一个线性的连续空间。例如，读请求（260，6）表示的是需要读取从扇区号为260的逻辑扇区开始，总共6个扇区。
	*large_lsn: channel下面有多少个subpage，即多少个sector。overprovide系数：SSD中并不是所有的空间都可以给用户使用，
	*比如32G的SSD可能有10%的空间保留下来留作他用，所以乘以1-provide
	***********************************************************************************************************/
	large_lsn = (int)((ssd->parameter->subpage_page*ssd->parameter->page_block*ssd->parameter->block_plane*ssd->parameter->plane_die*ssd->parameter->die_chip*ssd->parameter->chip_num)*(1 - ssd->parameter->overprovide));
	lsn = lsn%large_lsn;

	nearest_event_time = find_nearest_event(ssd);

	if (nearest_event_time == 0x7fffffffffffffff)
	{
		ssd->current_time = time_t;
		if (buffer_full_flag == 1)
		{
			fseek(ssd->tracefile, filepoint, 0);
			return -1;
		}
		else if (ssd->request_queue_length >= ssd->parameter->queue_length)
		{
			fseek(ssd->tracefile, filepoint, 0);
			return 0;
		}
	}
	else
	{
		if ( (nearest_event_time<time_t) || (buffer_full_flag == 1))
		{
			/*******************************************************************************
			*回滚，即如果没有把time_t赋给ssd->current_time，则trace文件已读的一条记录回滚
			*filepoint记录了执行fgets之前的文件指针位置，回滚到文件头+filepoint处
			*int fseek(FILE *stream, long offset, int fromwhere);函数设置文件指针stream的位置。
			*如果执行成功，stream将指向以fromwhere（偏移起始位置：文件头0，当前位置1，文件尾2）为基准，
			*偏移offset（指针偏移量）个字节的位置。如果执行失败(比如offset超过文件自身大小)，则不改变stream指向的位置。
			*文本文件只能采用文件头0的定位方式，本程序中打开文件方式是"r":以只读方式打开文本文件
			**********************************************************************************/
			fseek(ssd->tracefile, filepoint, 0);
			if (ssd->current_time <= nearest_event_time)
				ssd->current_time = nearest_event_time;
			return -1;
		}
		else
		{
			//两个条件，请求超过队列或者buff被阻塞
			if ( (ssd->request_queue_length >= ssd->parameter->queue_length)  ||  (buffer_full_flag == 1) )
			{
				fseek(ssd->tracefile, filepoint, 0);
				ssd->current_time = nearest_event_time;
				return -1;
			}
			else
			{
				ssd->current_time = time_t;
			}
		}
	}

	

	if (time_t < 0)
	{
		printf("error!\n");
		while (1){}
	}


	if (feof(ssd->tracefile))      //判断是否读完整个trace
	{
		request1 = NULL;
		trace_over_flag = 1;
		return 0;
	}

	request1 = (struct request*)malloc(sizeof(struct request));
	alloc_assert(request1, "request");
	memset(request1, 0, sizeof(struct request));

	request1->time = time_t;
	request1->lsn = lsn;
	request1->size = size;
	request1->operation = ope;
	request1->begin_time = time_t;
	request1->response_time = 0;
	request1->energy_consumption = 0;
	request1->next_node = NULL;
	request1->distri_flag = 0;              // indicate whether this request has been distributed already
	request1->subs = NULL;
	request1->need_distr_flag = NULL;
	request1->complete_lsn_count = 0;         //record the count of lsn served by buffer
	filepoint = ftell(ssd->tracefile);		// set the file point

	if (ssd->request_queue == NULL)          //The queue is empty
	{
		ssd->request_queue = request1;
		ssd->request_tail = request1;
		ssd->request_work = request1;
		ssd->request_queue_length++;
	}
	else
	{
		(ssd->request_tail)->next_node = request1;
		ssd->request_tail = request1;
		if (ssd->request_work == NULL)
			ssd->request_work = request1;
		ssd->request_queue_length++;
	}

	request_lz_count++;
	printf("request:%I64u\n", request_lz_count);
	//printf("%d\n", ssd->request_queue_length);

	/*
	if (request_lz_count == 3698863)
		printf("lz\n");
	*/

	if (request1->operation == 1)             //计算平均请求大小 1为读 0为写
	{
		ssd->ave_read_size = (ssd->ave_read_size*ssd->read_request_count + request1->size) / (ssd->read_request_count + 1);
	}
	else
	{
		ssd->ave_write_size = (ssd->ave_write_size*ssd->write_request_count + request1->size) / (ssd->write_request_count + 1);
	}


	filepoint = ftell(ssd->tracefile);
	fgets(buffer, 200, ssd->tracefile);    //寻找下一条请求的到达时间
	sscanf(buffer, "%I64u %d %d %d %d", &time_t, &device, &lsn, &size, &ope);
	ssd->next_request_time = time_t;
	fseek(ssd->tracefile, filepoint, 0);

	return 1;
}


/**********************************************************************************************************
*__int64 find_nearest_event(struct ssd_info *ssd)
*寻找所有子请求的最早到达的下个状态时间,首先看请求的下一个状态时间，如果请求的下个状态时间小于等于当前时间，
*说明请求被阻塞，需要查看channel或者对应die的下一状态时间。Int64是有符号 64 位整数数据类型，值类型表示值介于
*-2^63 ( -9,223,372,036,854,775,808)到2^63-1(+9,223,372,036,854,775,807 )之间的整数。存储空间占 8 字节。
*channel,die是事件向前推进的关键因素，三种情况可以使事件继续向前推进，channel，die分别回到idle状态，die中的
*读数据准备好了
***********************************************************************************************************/
__int64 find_nearest_event(struct ssd_info *ssd)
{
	unsigned int i, j;
	__int64 time = 0x7fffffffffffffff;
	__int64 time1 = 0x7fffffffffffffff;
	__int64 time2 = 0x7fffffffffffffff;

	for (i = 0; i<ssd->parameter->channel_number; i++)
	{
		if (ssd->channel_head[i].next_state == CHANNEL_IDLE)
			if (time1>ssd->channel_head[i].next_state_predict_time)
				if (ssd->channel_head[i].next_state_predict_time>ssd->current_time)
					time1 = ssd->channel_head[i].next_state_predict_time;
		for (j = 0; j<ssd->parameter->chip_channel[i]; j++)
		{
			if ((ssd->channel_head[i].chip_head[j].next_state == CHIP_IDLE) || (ssd->channel_head[i].chip_head[j].next_state == CHIP_DATA_TRANSFER))
				if (time2>ssd->channel_head[i].chip_head[j].next_state_predict_time)
					if (ssd->channel_head[i].chip_head[j].next_state_predict_time>ssd->current_time)
						time2 = ssd->channel_head[i].chip_head[j].next_state_predict_time;
		}
	}

	/*****************************************************************************************************
	*time为所有 A.下一状态为CHANNEL_IDLE且下一状态预计时间大于ssd当前时间的CHANNEL的下一状态预计时间
	*           B.下一状态为CHIP_IDLE且下一状态预计时间大于ssd当前时间的DIE的下一状态预计时间
	*		     C.下一状态为CHIP_DATA_TRANSFER且下一状态预计时间大于ssd当前时间的DIE的下一状态预计时间
	*CHIP_DATA_TRANSFER读准备好状态，数据已从介质传到了register，下一状态是从register传往buffer中的最小值
	*注意可能都没有满足要求的time，这时time返回0x7fffffffffffffff 。
	*****************************************************************************************************/
	time = (time1>time2) ? time2 : time1;
	return time;
}