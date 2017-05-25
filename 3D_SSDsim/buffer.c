/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName£º buffer.c
Author: Zuo Lu 		Version: 1.1	Date:2017/05/12
Description: 
buff layer: only contains data cache (minimum processing size for the sector, that is, unit = 512B), mapping table (page-level);

History:
<contributor>     <time>        <version>       <desc>									<e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim							617376665@qq.com
Zuo Lu			2017/05/12		  1.1			Support advanced commands:mutli plane   617376665@qq.com
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


/**********************************************************************************************************************************************
*Buff strategy:Blocking buff strategy
*1--first check the buffer is full, if dissatisfied, check whether the current request to put down the data, if so, put the current request,
*if not, then block the buffer;
*
*2--If buffer is blocked, select the replacement of the two ends of the page. If the two full page, then issued together to lift the buffer
*block; if a partial page 1 full page or 2 partial page, then issued a pre-read request, waiting for the completion of full page and then issued
*And then release the buffer block.
***********************************************************************************************************************************************/
struct ssd_info *buffer_management(struct ssd_info *ssd)
{
	unsigned int j, lsn, lpn, last_lpn, first_lpn, index, complete_flag = 0, state, full_page;
	unsigned int flag = 0, need_distb_flag, lsn_flag, flag1 = 1, active_region_flag = 0;
	struct request *new_request;
	struct buffer_group *buffer_node, key;
	unsigned int mask = 0, offset1 = 0, offset2 = 0;
	unsigned int page_type;
	unsigned int buff_free_count;
	unsigned int request_flag = 0;

	unsigned int lz_k;

#ifdef DEBUG
	printf("enter buffer_management,  current time:%I64u\n", ssd->current_time);
#endif

	ssd->dram->current_time = ssd->current_time;
	full_page = ~(0xffffffff << ssd->parameter->subpage_page);

	//new_request = ssd->request_tail;
	new_request = ssd->request_work;
	lsn = new_request->lsn;
	lpn = new_request->lsn / ssd->parameter->subpage_page;
	last_lpn = (new_request->lsn + new_request->size - 1) / ssd->parameter->subpage_page;
	first_lpn = new_request->lsn / ssd->parameter->subpage_page;   //Calculate lpn

	if (new_request->operation == READ)
	{

		//Int 32-bit, that is need to be_distr_flag is a bit-based status bit array
		new_request->need_distr_flag = (unsigned int*)malloc(sizeof(unsigned int)*((last_lpn - first_lpn + 1)*ssd->parameter->subpage_page / 32 + 1));
		alloc_assert(new_request->need_distr_flag, "new_request->need_distr_flag");
		memset(new_request->need_distr_flag, 0, sizeof(unsigned int)*((last_lpn - first_lpn + 1)*ssd->parameter->subpage_page / 32 + 1));

		while (lpn <= last_lpn)
		{
			/************************************************************************************************
			*need_distb_flag indicates whether the need to perform distribution function, 1 said the need to implement, buffer not, 0 that does not need to
			*************************************************************************************************/
			need_distb_flag = full_page;   //need_distb_flag flags the status bits of the current lpn subpage
			key.group = lpn;
			buffer_node = (struct buffer_group*)avlTreeFind(ssd->dram->buffer, (TREE_NODE *)&key);		

			while ((buffer_node != NULL) && (lsn<(lpn + 1)*ssd->parameter->subpage_page) && (lsn <= (new_request->lsn + new_request->size - 1)))
			{
				lsn_flag = full_page;
				mask = 1 << (lsn%ssd->parameter->subpage_page);         
				//if (mask>31)
				if (mask > 0x80000000)
				{
					printf("the subpage number is larger than 32!add some cases");   //Note that the maximum number of sub pages can not beyong 32
					getchar();
				}
				else if ((buffer_node->stored & mask) == mask)
				{
					flag = 1;
					lsn_flag = lsn_flag&(~mask);   //The current hit in the buff of the sub-page state is modified to 0, stored in lsn_flag on
				}

				if (flag == 1)
				{	//If the buffer node is not in the buffer of the head, the need to mention the node to the head, to achieve the LRU algorithm, this is a two-way queue.	       		
					if (ssd->dram->buffer->buffer_head != buffer_node)
					{
						if (ssd->dram->buffer->buffer_tail == buffer_node)
						{
							buffer_node->LRU_link_pre->LRU_link_next = NULL;
							ssd->dram->buffer->buffer_tail = buffer_node->LRU_link_pre;
						}
						else
						{
							buffer_node->LRU_link_pre->LRU_link_next = buffer_node->LRU_link_next;
							buffer_node->LRU_link_next->LRU_link_pre = buffer_node->LRU_link_pre;
						}
						buffer_node->LRU_link_next = ssd->dram->buffer->buffer_head;
						ssd->dram->buffer->buffer_head->LRU_link_pre = buffer_node;
						buffer_node->LRU_link_pre = NULL;
						ssd->dram->buffer->buffer_head = buffer_node;
					}
					new_request->complete_lsn_count++;
				}
				else if (flag == 0)
				{
					//ssd->dram->buffer->read_miss_hit++;
				}

				need_distb_flag = need_distb_flag&lsn_flag;   //Save the current page status of lpn, 0 for hits, and 1 for miss
				flag = 0;
				lsn++;
			}

			if (need_distb_flag == 0x00000000)
				ssd->dram->buffer->read_hit++;
			else
				ssd->dram->buffer->read_miss_hit++;

			index = (lpn - first_lpn) / (32 / ssd->parameter->subpage_page);  //Index that need_distr_flag there are many int array index, index is the index of this array
			new_request->need_distr_flag[index] = new_request->need_distr_flag[index] | (need_distb_flag << (((lpn - first_lpn) % (32 / ssd->parameter->subpage_page))*ssd->parameter->subpage_page));
			lpn++;

		}
	}
	else if (new_request->operation == WRITE)
	{
		//if (new_request->lsn == 725320)
			//getchar();

		while (new_request->cmplt_flag == 0)
		{
			//First check all lpn of the request
			lpn = new_request->lsn / ssd->parameter->subpage_page;
			buff_free_count = ssd->dram->buffer->max_buffer_sector - ssd->dram->buffer->buffer_sector_count;
			while (lpn <= last_lpn)
			{
				//In this while inside, first check again buff this time can be put down, if you can put down on the insertbuff, can not block the buff
				key.group = lpn;
				buffer_node = (struct buffer_group*)avlTreeFind(ssd->dram->buffer, (TREE_NODE *)&key);	    //Check the buff in the hit
				if (buffer_node == NULL)
				{
					if (buff_free_count >= ssd->parameter->subpage_page)
					{
						buff_free_count = buff_free_count - ssd->parameter->subpage_page;
					}
					else  //Buff is full, blocking buff, break
					{
						ssd->buffer_full_flag = 1;
						break;
					}
				}
				lpn++;
			}

			if (ssd->buffer_full_flag == 0)
			{
				//Buff at this time is not full, write datto the buff 
				//lz_k++;
				lpn = new_request->lsn / ssd->parameter->subpage_page;
				while (lpn <= last_lpn)
				{
					need_distb_flag = full_page;
					mask = ~(0xffffffff << (ssd->parameter->subpage_page));
					state = mask;

					if (lpn == first_lpn)
					{
						offset1 = ssd->parameter->subpage_page - ((lpn + 1)*ssd->parameter->subpage_page - new_request->lsn);
						state = state&(0xffffffff << offset1);
					}
					if (lpn == last_lpn)
					{
						offset2 = ssd->parameter->subpage_page - ((lpn + 1)*ssd->parameter->subpage_page - (new_request->lsn + new_request->size));
						state = state&(~(0xffffffff << offset2));
					}

					//calculate the type of lpn, which is full page or partial page
					if ((state&ssd->dram->map->map_entry[lpn].state) != ssd->dram->map->map_entry[lpn].state)
						page_type = 1;
					else
						page_type = 0;

					ssd = insert2buffer(ssd, lpn, state, page_type, NULL, new_request);
					lpn++;
				}
				lpn = 0;
				new_request->cmplt_flag = 1;			//The request has been executed
				//printf("write insert,%d\n", lz_k);
				//if (lz_k == 322931)
					//getchar();

			}
			else
			{
				//Buff full, from the end of the buff to replace the two requests getout2buffer
				getout2buffer(ssd, NULL, new_request);
				if (ssd->buffer_full_flag == 1)
					break;

				new_request->cmplt_flag = 0;			//The request was not executed
				//printf("write 2 lpn\n");
			}
		}
	}

	
	complete_flag = 1;
	if (new_request->operation == READ)
	{
		for (j = 0; j <= (last_lpn - first_lpn + 1)*ssd->parameter->subpage_page / 32; j++)
		{
			if (new_request->need_distr_flag[j] != 0)
			{
				complete_flag = 0;
			}
		}
	}

	/*************************************************************
	*If the request has been fully served by the buffer, the request 
	*can be directly responded to the output
	*Here assume that dram's service time is 1000ns
	**************************************************************/
	if ((complete_flag == 1) && (new_request->subs == NULL))
	{
		new_request->cmplt_flag = 1;
		new_request->begin_time = ssd->current_time;
		new_request->response_time = ssd->current_time + 1000;
	}

	return ssd;
}



struct ssd_info * getout2buffer(struct ssd_info *ssd, struct sub_request *sub, struct request *req)
{
	unsigned int req_write_counts = 0;
	struct buffer_group *pt;
	unsigned int sub_req_state = 0, sub_req_size = 0, sub_req_lpn = 0, sub_req_type = 0;
	struct sub_request *sub_req = NULL;


	//tail node replacement
	pt = ssd->dram->buffer->buffer_tail;
	if (pt->page_type == 0 && pt->LRU_link_pre->page_type == 0)
	{
		//Send two full pages and delete the two nodes
		for (req_write_counts = 0; req_write_counts < 2; req_write_counts++)
		{
			//printf("write\n");
			ssd->dram->buffer->write_miss_hit++;
			pt = ssd->dram->buffer->buffer_tail;

			sub_req_state = pt->stored;
			sub_req_size = size(pt->stored);
			sub_req_lpn = pt->group;
			sub_req_type = pt->page_type;
			sub_req = NULL;
			sub_req = creat_sub_request(ssd, sub_req_lpn, sub_req_size, sub_req_state, sub_req_type, req, WRITE);

			//Delete the node
			ssd->dram->buffer->buffer_sector_count = ssd->dram->buffer->buffer_sector_count - ssd->parameter->subpage_page;

			if (pt == ssd->dram->buffer->buffer_tail)
			{
				if (ssd->dram->buffer->buffer_head->LRU_link_next == NULL){
					ssd->dram->buffer->buffer_head = NULL;
					ssd->dram->buffer->buffer_tail = NULL;
				}
				else{
					ssd->dram->buffer->buffer_tail = pt->LRU_link_pre;
					ssd->dram->buffer->buffer_tail->LRU_link_next = NULL;
				}
			}
			else
			{
				printf("buffer_tail delete failed\n");
				getchar();
			}
			
			avlTreeDel(ssd->dram->buffer, (TREE_NODE *)pt);
			pt->LRU_link_next = NULL;
			pt->LRU_link_pre = NULL;
			AVL_TREENODE_FREE(ssd->dram->buffer, (TREE_NODE *)pt);
			pt = NULL;
		}

		//Cancel the blocking buff
		ssd->buffer_full_flag = 0;
	}
	else if (pt->page_type == 1)
	{
		//Send a pre read
		//printf("update read\n");
		//getchar();
		sub_req_state = pt->stored;
		sub_req_size = size(pt->stored);
		sub_req_lpn = pt->group;
		sub_req_type = pt->page_type;
		sub_req = NULL;
		sub_req = creat_sub_request(ssd, sub_req_lpn, sub_req_size, sub_req_state, sub_req_type, req, UPDATE_READ);
	}
	else if (pt->LRU_link_pre->page_type == 1)
	{
		//Send a pre read
		//printf("update read\n");
		//getchar();
		sub_req_state = pt->LRU_link_pre->stored;
		sub_req_size = size(pt->LRU_link_pre->stored);
		sub_req_lpn = pt->LRU_link_pre->group;
		sub_req_type = pt->LRU_link_pre->page_type;
		sub_req = NULL;
		sub_req = creat_sub_request(ssd, sub_req_lpn, sub_req_size, sub_req_state, sub_req_type, req, UPDATE_READ);
	}
	else
	{
		//Send two pre read
		//printf("update read\n");
		//getchar();
		for (req_write_counts = 0; req_write_counts < 2; req_write_counts++)
		{
			sub_req_state = pt->stored;
			sub_req_size = size(pt->stored);
			sub_req_lpn = pt->group;
			sub_req_type = pt->page_type;
			sub_req = NULL;
			sub_req = creat_sub_request(ssd, sub_req_lpn, sub_req_size, sub_req_state, sub_req_type, req, UPDATE_READ);

			pt = pt->LRU_link_pre;
		}
	}
	return ssd;
}


/*******************************************************************************
*The function is to write data to the buffer,Called by buffer_management()
********************************************************************************/
struct ssd_info * insert2buffer(struct ssd_info *ssd, unsigned int lpn, int state,unsigned int page_type, struct sub_request *sub, struct request *req )
{
	int write_back_count, flag = 0;                                                             
	unsigned int i, lsn, add_flag, hit_flag, sector_count, active_region_flag = 0;
	unsigned int free_sector;
	unsigned int page_size=0;
	struct buffer_group *buffer_node = NULL, *new_node = NULL, key;
	struct sub_request *sub_req = NULL, *update = NULL;
	unsigned int req_write_counts = 0;
	unsigned int full_page_flag = 0;
	unsigned int page_flag = 0;


#ifdef DEBUG
	printf("enter insert2buffer,  current time:%I64u, lpn:%d, state:%d,\n", ssd->current_time, lpn, state);
#endif

	sector_count = size(state);                                                                /*need to write the number of sectors of the buffer*/
	key.group = lpn;
	buffer_node = (struct buffer_group*)avlTreeFind(ssd->dram->buffer, (TREE_NODE *)&key);    /*Look for the buffer node in the balanced binary tree*/
	page_size = ssd->parameter->subpage_page;
	flag = 0;

	if (buffer_node == NULL)
	{
		free_sector = ssd->dram->buffer->max_buffer_sector - ssd->dram->buffer->buffer_sector_count;
		if (free_sector >= page_size)
		{
			flag = 1;																			//Can be added directly into the buff, that the request completed
			//ssd->dram->buffer->write_hit++;
		}
		if (flag == 0)																		    //not hit, and at this time buff in the remaining sector space is not enough to replace, replace lpn to flash up,
		{
			printf("Error! insert2buff error,no free space\n");
			getchar();
		}

		/******************************************************************************
		*No hits, write this lpn into buff
		*Generate a buffer node, according to the situation of this page were assigned 
		*to each member, added to the head and the binary tree
		*******************************************************************************/
		new_node = NULL;
		new_node = (struct buffer_group *)malloc(sizeof(struct buffer_group));
		alloc_assert(new_node, "buffer_group_node");
		memset(new_node, 0, sizeof(struct buffer_group));

		new_node->group = lpn;
		new_node->stored = state;
		new_node->dirty_clean = state;
		new_node->page_type = page_type;

		new_node->LRU_link_pre = NULL;
		new_node->LRU_link_next = ssd->dram->buffer->buffer_head;
		if (ssd->dram->buffer->buffer_head != NULL){
			ssd->dram->buffer->buffer_head->LRU_link_pre = new_node;
		}
		else{
			ssd->dram->buffer->buffer_tail = new_node;
		}
		ssd->dram->buffer->buffer_head = new_node;
		new_node->LRU_link_pre = NULL;
		avlTreeAdd(ssd->dram->buffer, (TREE_NODE *)new_node);

		ssd->dram->buffer->buffer_sector_count += page_size;

	}
	/****************************************************************************************
	*In the buffer hit the situation, the full hit is a direct return, part of the hit, merge the sector and return
	*****************************************************************************************/
	else
	{
		if (state == buffer_node->stored)
		{
			ssd->dram->buffer->write_hit++;
		}
		else
		{
			buffer_node->stored = buffer_node->stored | state;
			buffer_node->dirty_clean = buffer_node->dirty_clean | state;

			//After the merger,determine the existence of buff node node type
			if ((buffer_node->stored&ssd->dram->map->map_entry[lpn].state) != ssd->dram->map->map_entry[lpn].state)
				buffer_node->page_type = 1;
			else
				buffer_node->page_type = 0;

			ssd->dram->buffer->write_hit++;
		}
		//Hit lpn, move the node to the head
		if (ssd->dram->buffer->buffer_head != buffer_node)
		{
			if (ssd->dram->buffer->buffer_tail == buffer_node)
			{
				ssd->dram->buffer->buffer_tail = buffer_node->LRU_link_pre;
				buffer_node->LRU_link_pre->LRU_link_next = NULL;
			}
			else if (buffer_node != ssd->dram->buffer->buffer_head)
			{
				buffer_node->LRU_link_pre->LRU_link_next = buffer_node->LRU_link_next;
				buffer_node->LRU_link_next->LRU_link_pre = buffer_node->LRU_link_pre;
			}
			buffer_node->LRU_link_next = ssd->dram->buffer->buffer_head;
			ssd->dram->buffer->buffer_head->LRU_link_pre = buffer_node;
			buffer_node->LRU_link_pre = NULL;
			ssd->dram->buffer->buffer_head = buffer_node;
		}
		req->complete_lsn_count++;
	}
	//printf("bufff_count,%d\n",avlTreeCount(ssd->dram->buffer));
	return ssd;
}

/**********************************************************************************
*Read requests allocate sub-request functions, decompose each request into sub-requests 
*based on the request queue and buffer hit check, hang the sub request queue on the 
*channel, and the different channel has its own sub request queue
**********************************************************************************/

struct ssd_info *distribute(struct ssd_info *ssd)
{
	unsigned int start, end, first_lsn, last_lsn, lpn, flag = 0, flag_attached = 0, full_page;
	unsigned int j, k, sub_size;
	int i = 0;
	struct request *req;
	struct sub_request *sub;
	int* complt;

#ifdef DEBUG
	printf("enter distribute,  current time:%I64u\n", ssd->current_time);
#endif
	full_page = ~(0xffffffff << ssd->parameter->subpage_page);

	//req = ssd->request_tail;
	req = ssd->request_work;
	if (req->response_time != 0){
		return ssd;
	}
	if (req->operation == WRITE)
	{
		return ssd;
	}

	if (req != NULL)
	{
		if (req->distri_flag == 0)
		{
			if (req->complete_lsn_count != ssd->request_work->size)
			{
				first_lsn = req->lsn;
				last_lsn = first_lsn + req->size;
				complt = req->need_distr_flag;    //Sub pages that have been completed in buff

				//Start, end Indicates the start and end of the request in units of sub pages
				start = first_lsn - first_lsn % ssd->parameter->subpage_page;
				end = (last_lsn / ssd->parameter->subpage_page + 1) * ssd->parameter->subpage_page;

				//ow many sub pages are requested, that is, how many int types are marked
				i = (end - start) / 32;


				while (i >= 0)
				{
					/*************************************************************************************
					*Each bit of a 32-bit integer data represents a subpage, 32 / ssd-> parameter-> subpage_page 
					*indicates how many pages are, and the status of each page is stored in req-> need_distr_flag, 
					*that is, Complt, by comparing complt each and full_page, you can know whether this page is 
					*processed. Create a subquery with the creat_sub_request function if no processing is done.
					*************************************************************************************/
					for (j = 0; j<32 / ssd->parameter->subpage_page; j++)
					{
						k = (complt[((end - start) / 32 - i)] >> (ssd->parameter->subpage_page*j)) & full_page;   //buff inserted in the bit state of the opposite, the corresponding lpn bit state out
						if (k != 0)    //Note that the current lpn did not completely hit in the buffe, the need for a read operation
						{
							lpn = start / ssd->parameter->subpage_page + ((end - start) / 32 - i) * 32 / ssd->parameter->subpage_page + j;
							sub_size = transfer_size(ssd, k, lpn, req);  //Sub_size is the number of pages that need to go to the flash
							if (sub_size == 0)
							{
								continue;
							}
							else
							{
								printf("normal read\n");
								sub = creat_sub_request(ssd, lpn, sub_size, 0, 0, req, req->operation);
							}
						}
					}
					i = i - 1;
				}

			}
			else
			{
				req->begin_time = ssd->current_time;
				req->response_time = ssd->current_time + 1000;
			}

		}
	}
	req->cmplt_flag = 1; 
	return ssd;
}


/*********************************************************************************************
*The no_buffer_distribute () function is processed when ssd has no dram£¬
*This is no need to read and write requests in the buffer inside the search, directly use the 
*creat_sub_request () function to create sub-request, and then deal with.
*********************************************************************************************/
struct ssd_info *no_buffer_distribute(struct ssd_info *ssd)
{
	unsigned int lsn, lpn, last_lpn, first_lpn, complete_flag = 0, state;
	unsigned int flag = 0, flag1 = 1, active_region_flag = 0;           //to indicate the lsn is hitted or not
	struct request *req = NULL;
	struct sub_request *sub = NULL, *sub_r = NULL, *update = NULL;
	struct local *loc = NULL;
	struct channel_info *p_ch = NULL;


	unsigned int mask = 0;
	unsigned int offset1 = 0, offset2 = 0;
	unsigned int sub_size = 0;
	unsigned int sub_state = 0;

	ssd->dram->current_time = ssd->current_time;
	//req = ssd->request_tail;
	req = ssd->request_work;
	lsn = req->lsn;
	lpn = req->lsn / ssd->parameter->subpage_page;
	last_lpn = (req->lsn + req->size - 1) / ssd->parameter->subpage_page;
	first_lpn = req->lsn / ssd->parameter->subpage_page;

	if (req->operation == READ)
	{
		while (lpn <= last_lpn)
		{
			sub_state = (ssd->dram->map->map_entry[lpn].state & 0x7fffffff);
			sub_size = size(sub_state);
			sub = creat_sub_request(ssd, lpn, sub_size, sub_state,0, req, req->operation);
			lpn++;
		}
	}
	else if (req->operation == WRITE)
	{
		while (lpn <= last_lpn)
		{
			mask = ~(0xffffffff << (ssd->parameter->subpage_page));
			state = mask;
			if (lpn == first_lpn)
			{
				offset1 = ssd->parameter->subpage_page - ((lpn + 1)*ssd->parameter->subpage_page - req->lsn);
				state = state&(0xffffffff << offset1);
			}
			if (lpn == last_lpn)
			{
				offset2 = ssd->parameter->subpage_page - ((lpn + 1)*ssd->parameter->subpage_page - (req->lsn + req->size));
				state = state&(~(0xffffffff << offset2));
			}
			sub_size = size(state);

			sub = creat_sub_request(ssd, lpn, sub_size, state,0, req, req->operation);
			lpn++;
		}
	}

	return ssd;
}

/****************************************************************************************************************
*The function of the transfer_size () is to calculate the size of the sub request that needs to be processed
*The first special case of first_lpn and last_lpn is handled in the function, since these two cases are likely 
*not to deal with a whole page but to deal with a part of the page, since lsn may not be the first subpage of a page.
*******************************************************************************************************************/
unsigned int transfer_size(struct ssd_info *ssd, int need_distribute, unsigned int lpn, struct request *req)
{
	unsigned int first_lpn, last_lpn, state, trans_size;
	unsigned int mask = 0, offset1 = 0, offset2 = 0;

	first_lpn = req->lsn / ssd->parameter->subpage_page;
	last_lpn = (req->lsn + req->size - 1) / ssd->parameter->subpage_page;

	mask = ~(0xffffffff << (ssd->parameter->subpage_page));
	state = mask;
	if (lpn == first_lpn)
	{
		offset1 = ssd->parameter->subpage_page - ((lpn + 1)*ssd->parameter->subpage_page - req->lsn);
		state = state&(0xffffffff << offset1);
	}
	if (lpn == last_lpn)
	{
		offset2 = ssd->parameter->subpage_page - ((lpn + 1)*ssd->parameter->subpage_page - (req->lsn + req->size));
		state = state&(~(0xffffffff << offset2));
	}

	trans_size = size(state&need_distribute);

	return trans_size;
}



/***********************************************************************************
*According to the status of each page to calculate the number of each need to deal 
*with the number of sub-pages, that is, a sub-request to deal with the number of pages
************************************************************************************/
unsigned int size(unsigned int stored)
{
	unsigned int i, total = 0, mask = 0x80000000;

#ifdef DEBUG
	printf("enter size\n");
#endif
	for (i = 1; i <= 32; i++)
	{
		if (stored & mask) total++;     //The total count indicates that the flag is 0 in the subpage
		stored <<= 1;
	}
#ifdef DEBUG
	printf("leave size\n");
#endif
	return total;
}


/**************************************************************
this function is to create sub_request based on lpn, size, state
****************************************************************/
struct sub_request * creat_sub_request(struct ssd_info * ssd, unsigned int lpn, int page_size, unsigned int state,unsigned int page_type, struct request *req, unsigned int operation)
{
	struct sub_request* sub = NULL, *sub_r = NULL;
	struct channel_info * p_ch = NULL;
	struct local * loc = NULL;
	unsigned int flag = 0;
	struct sub_request * update = NULL;
	struct local *location = NULL;

	sub = (struct sub_request*)malloc(sizeof(struct sub_request));                      
	alloc_assert(sub, "sub_request");
	memset(sub, 0, sizeof(struct sub_request));

	if (sub == NULL)
	{
		return NULL;
	}
	sub->location = NULL;
	sub->next_node = NULL;
	sub->next_subs = NULL;
	sub->update = NULL;


	//hang the sub request on the current request
	if (req != NULL)
	{
		sub->next_subs = req->subs;
		req->subs = sub;
	}

	/*************************************************************************************
	*In the case of a read operation, it is very important to determine in advance whether 
	*there is any such request in the sub request queue, and the new sub request will not 
	*be executed again, and the new sub-request will be given directly
	**************************************************************************************/
	if (operation == READ)
	{
		loc = find_location(ssd, ssd->dram->map->map_entry[lpn].pn);
		sub->location = loc;
		sub->begin_time = ssd->current_time;
		sub->current_state = SR_WAIT;
		sub->current_time = 0x7fffffffffffffff;
		sub->next_state = SR_R_C_A_TRANSFER;
		sub->next_state_predict_time = 0x7fffffffffffffff;
		sub->lpn = lpn;
		sub->size = page_size;                                                             /*It is necessary to calculate the request size of the sub request*/

		p_ch = &ssd->channel_head[loc->channel];
		sub->ppn = ssd->dram->map->map_entry[lpn].pn;
		sub->operation = READ;
		sub->state = (ssd->dram->map->map_entry[lpn].state & 0x7fffffff);
		sub->update_read_flag = 0;

		sub_r = p_ch->subs_r_head;                                                      /*To determine whether there is any such request in the sub request queue, and to assign the new sub-request directly to the completion*/
		flag = 0;
		while (sub_r != NULL)
		{
			if (sub_r->ppn == sub->ppn)
			{
				flag = 1;
				break;
			}
			sub_r = sub_r->next_node;
		}
		if (flag == 0)
		{
			if (p_ch->subs_r_tail != NULL)
			{
				p_ch->subs_r_tail->next_node = sub;
				p_ch->subs_r_tail = sub;
			}
			else
			{
				p_ch->subs_r_head = sub;
				p_ch->subs_r_tail = sub;
			}
		}
		else
		{
			sub->current_state = SR_R_DATA_TRANSFER;
			sub->current_time = ssd->current_time;
			sub->next_state = SR_COMPLETE;
			sub->next_state_predict_time = ssd->current_time + 1000;
			sub->complete_time = ssd->current_time + 1000;
		}
	}
	/*************************************************************************************
	*Write request, you need to use the function allocate_location (ssd, sub) to deal with 
	*static allocation and dynamic allocation
	**************************************************************************************/
	else if (operation == WRITE)
	{
		sub->ppn = 0;
		sub->operation = WRITE;
		sub->location = (struct local *)malloc(sizeof(struct local));
		alloc_assert(sub->location, "sub->location");
		memset(sub->location, 0, sizeof(struct local));

		sub->current_state = SR_WAIT;
		sub->current_time = ssd->current_time;
		sub->lpn = lpn;
		sub->size = page_size;
		sub->state = state;
		sub->begin_time = ssd->current_time;
		sub->update_read_flag = 0;

		if (allocate_location(ssd, sub) == ERROR)
		{
			free(sub->location);
			sub->location = NULL;
			free(sub);
			sub = NULL;
			printf("allocate_location error \n");
			return NULL;
		}
	}
	else if (operation == UPDATE_READ)
	{
		//The partial page needs to generate an update read
		ssd->update_read_count++;

		if (sub == NULL)
		{
			return NULL;
		}

		location = find_location(ssd, ssd->dram->map->map_entry[lpn].pn);
		sub->location = location;
		sub->begin_time = ssd->current_time;
		sub->current_state = SR_WAIT;
		sub->current_time = 0x7fffffffffffffff;
		sub->next_state = SR_R_C_A_TRANSFER;
		sub->next_state_predict_time = 0x7fffffffffffffff;
		sub->lpn = lpn;
		sub->state = ((ssd->dram->map->map_entry[lpn].state^state) & 0x7fffffff);
		sub->size = size(sub->state);
		sub->ppn = ssd->dram->map->map_entry[lpn].pn;
		sub->operation = READ;
		sub->update_read_flag = page_type;

		if (sub->update_read_flag == 0)
		{
			printf("not in partial page!\n");
			getchar();
		}

		if (ssd->channel_head[location->channel].subs_r_tail != NULL)            /*Generates a new read request and hangs the end of the subs_r_tail queue of the channel*/
		{
			ssd->channel_head[location->channel].subs_r_tail->next_node = sub;
			ssd->channel_head[location->channel].subs_r_tail = sub;
		}
		else
		{
			ssd->channel_head[location->channel].subs_r_tail = sub;
			ssd->channel_head[location->channel].subs_r_head = sub;
		}
	}
	else
	{
		free(sub->location);
		sub->location = NULL;
		free(sub);
		sub = NULL;
		printf("\nERROR ! Unexpected command.\n");
		return NULL;
	}
	return sub;
}

/***************************************
*Write request dynamic allocation mount
***************************************/
Status allocate_location(struct ssd_info * ssd, struct sub_request *sub_req)
{
	struct sub_request * update = NULL;
	unsigned int channel_num = 0, chip_num = 0, die_num = 0, plane_num = 0;
	struct local *location = NULL;

	channel_num = ssd->parameter->channel_number;
	chip_num = ssd->parameter->chip_channel[0];
	die_num = ssd->parameter->die_chip;
	plane_num = ssd->parameter->plane_die;

	if (ssd->parameter->allocation_scheme == 0)                                          
	{
		if (ssd->parameter->dynamic_allocation == 0)
		{
			sub_req->location->channel = -1;
			sub_req->location->chip = -1;
			sub_req->location->die = -1;
			sub_req->location->plane = -1;
			sub_req->location->block = -1;
			sub_req->location->page = -1;        //Mounted on ssdinfo, because the whole dynamic allocation, do not know which specific mount to the channel above

			if (sub_req != NULL)	    
			{
				if (ssd->subs_w_tail != NULL)
				{
					ssd->subs_w_tail->next_node = sub_req;
					ssd->subs_w_tail = sub_req;
				}
				else
				{
					ssd->subs_w_tail = sub_req;
					ssd->subs_w_head = sub_req;
				}
			}
			else
			{
				return ERROR;
			}
		}
	}
	return SUCCESS;
}