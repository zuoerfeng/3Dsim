/*****************************************************************************************************************************
This is a project on 3D_SSDsim, based on ssdsim under the framework of the completion of structures, the main function:
1.Support for 3D commands, for example:mutli plane\interleave\copyback\program suspend/Resume..etc
2.Multi - level parallel simulation
3.Clear hierarchical interface
4.4-layer structure

FileName： ssd.c
Author: Zuo Lu 		Version: 1.0	Date:2017/04/06
Description:
Initialization layer: complete ssd organizational data structure, request queue creation and memory space initialization

History:
<contributor>     <time>        <version>       <desc>                   <e-mail>
Zuo Lu	        2017/04/06	      1.0		    Creat 3D_SSDsim       617376665@qq.com

*****************************************************************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>
#include "avlTree.h"

#define SECTOR 512

#define DYNAMIC_ALLOCATION 0
#define STATIC_ALLOCATION 1

#define INTERLEAVE 0
#define TWO_PLANE 1

#define NORMAL    2
#define INTERLEAVE_TWO_PLANE 3
#define COPY_BACK	4

#define AD_RANDOM 1     //random
#define AD_COPYBACK 2	//copyback
#define AD_TWOPLANE 4   //mutli plane write
#define AD_INTERLEAVE 8  //interleave
#define AD_TWOPLANE_READ 16  //mutli plane read

#define READ 1
#define WRITE 0
#define UPDATE_READ 2

/*********************************all states of each objects************************************************
*Defines the channel idle, command address transmission, data transmission, transmission, and other states
*And chip free, write busy, read busy, command address transfer, data transfer, erase busy, copyback busy, 
*other status , And read and write requests (sub) wait, read command address transfer, read, read data transfer, 
*write command address transfer, write data transfer, write transfer, completion and other status
************************************************************************************************************/

#define CHANNEL_IDLE 000
#define CHANNEL_C_A_TRANSFER 3
#define CHANNEL_GC 4           
#define CHANNEL_DATA_TRANSFER 7
#define CHANNEL_TRANSFER 8
#define CHANNEL_UNKNOWN 9

#define CHIP_IDLE 100
#define CHIP_WRITE_BUSY 101
#define CHIP_READ_BUSY 102
#define CHIP_C_A_TRANSFER 103
#define CHIP_DATA_TRANSFER 107
#define CHIP_WAIT 108
#define CHIP_ERASE_BUSY 109
#define CHIP_COPYBACK_BUSY 110
#define UNKNOWN 111

#define SR_WAIT 200                 
#define SR_R_C_A_TRANSFER 201
#define SR_R_READ 202
#define SR_R_DATA_TRANSFER 203
#define SR_W_C_A_TRANSFER 204
#define SR_W_DATA_TRANSFER 205
#define SR_W_TRANSFER 206
#define SR_COMPLETE 299

#define REQUEST_IN 300         //Next request arrival time
#define OUTPUT 301             //The next time the data is output

#define GC_WAIT 400
#define GC_ERASE_C_A 401
#define GC_COPY_BACK 402
#define GC_COMPLETE 403
#define GC_INTERRUPT 0
#define GC_UNINTERRUPT 1

#define CHANNEL(lsn) (lsn&0x0000)>>16      
#define chip(lsn) (lsn&0x0000)>>16 
#define die(lsn) (lsn&0x0000)>>16 
#define PLANE(lsn) (lsn&0x0000)>>16 
#define BLOKC(lsn) (lsn&0x0000)>>16 
#define PAGE(lsn) (lsn&0x0000)>>16 
#define SUBPAGE(lsn) (lsn&0x0000)>>16  

#define PG_SUB 0xffffffff			

/*************************************************************************
*Function result status code
*Status is the function type ,the value is the function result status code
**************************************************************************/
#define TRUE		1
#define FALSE		0
#define SUCCESS		1
#define FAILURE		0
#define ERROR		-1
#define INFEASIBLE	-2
#define OVERFLOW	-3
typedef int Status;     

struct ac_time_characteristics{
	int tPROG;     //program time
	int tDBSY;     //bummy busy time for two-plane program
	int tBERS;     //block erase time
	int tCLS;      //CLE setup time
	int tCLH;      //CLE hold time
	int tCS;       //CE setup time
	int tCH;       //CE hold time
	int tWP;       //WE pulse width
	int tALS;      //ALE setup time
	int tALH;      //ALE hold time
	int tDS;       //data setup time
	int tDH;       //data hold time
	int tWC;       //write cycle time
	int tWH;       //WE high hold time
	int tADL;      //address to data loading time
	int tR;        //data transfer from cell to register
	int tAR;       //ALE to RE delay
	int tCLR;      //CLE to RE delay
	int tRR;       //ready to RE low
	int tRP;       //RE pulse width
	int tWB;       //WE high to busy
	int tRC;       //read cycle time
	int tREA;      //RE access time
	int tCEA;      //CE access time
	int tRHZ;      //RE high to output hi-z
	int tCHZ;      //CE high to output hi-z
	int tRHOH;     //RE high to output hold
	int tRLOH;     //RE low to output hold
	int tCOH;      //CE high to output hold
	int tREH;      //RE high to output time
	int tIR;       //output hi-z to RE low
	int tRHW;      //RE high to WE low
	int tWHR;      //WE high to RE low
	int tRST;      //device resetting time
}ac_timing;


struct ssd_info{ 
	//Global variable
	int make_age_free_page;				 //The number of free pages after make_aged
	int buffer_full_flag;				 //buffer blocking flag:0--unblocking , 1-- blocking
	int trace_over_flag;				 //the end of trace flag:0-- not ending ,1--ending
	__int64 request_lz_count;			 //trace request count

	__int64 current_time;                //Record system time
	__int64 next_request_time;
	unsigned int real_time_subreq;       //Record the number of real-time write requests, used in the full dynamic allocation, channel priority situation

	int flag;
	int active_flag;                     //Record the active write is blocked, if found plunger, need to move forward time, 0 that there is no block, 1 that is blocked, need to move forward time
	unsigned int page;

	unsigned int token;                  //In the dynamic allocation, in order to prevent each assignment in the first channel need to maintain a token, each time from the token refers to the location of the distribution
	unsigned int gc_request;             //Recorded in the SSD, the current moment how many gc operation request

	__int64 write_avg;                   //Record the time to calculate the average response time for the write request
	__int64 read_avg;                    //Record the time to calculate the average response time for the read request

	unsigned int min_lsn;
	unsigned int max_lsn;

	unsigned long read_count;
	unsigned long update_read_count;      //Record the number of updates read
	unsigned long gc_read_count;		  //Record gc caused by the read operation

	unsigned long program_count;
	unsigned long pre_all_write;		 //Record preprocessing write operation
	unsigned long update_write_count;	 //Record the number of updates write
	unsigned long gc_write_count;		 //Record gc caused by the write operation

	unsigned long erase_count;
	unsigned long direct_erase_count;    //Record invalid blocks that are directly erased

	//Advanced command read and write erase statistics
	unsigned long m_plane_read_count;
	unsigned long m_plane_prog_count;
	unsigned long interleave_count;
	unsigned long interleave_read_count;
	unsigned long inter_mplane_count;
	unsigned long inter_mplane_prog_count;
	unsigned long interleave_erase_count;
	unsigned long mplane_erase_conut;
	unsigned long interleave_mplane_erase_count;
	unsigned long gc_copy_back;
	unsigned long copy_back_count;

	unsigned long write_flash_count;     //The actual write to the flash
	unsigned long waste_page_count;      //Record the page waste due to restrictions on advanced commands

	unsigned long write_request_count;    //Record the number of write operations
	unsigned long read_request_count;     //Record the number of read operations
	
	float ave_read_size;
	float ave_write_size;
	unsigned int request_queue_length;
	
	char parameterfilename[30];
	char tracefilename[30];
	char outputfilename[30];
	char statisticfilename[30];
	char statisticfilename2[30];

	FILE * outputfile;
	FILE * tracefile;
	FILE * statisticfile;
	FILE * statisticfile2;

    struct parameter_value *parameter;   //SSD parameter
	struct dram_info *dram;
	struct request *request_queue;       //dynamic request queue
	struct request *request_head;		 // the head of the request queue
	struct request *request_tail;	     // the tail of the request queue
	struct request *request_work;		 // the work point of the request queue

	struct sub_request *subs_w_head;     //When using the full dynamic allocation, the first hanging on the ssd, etc. into the process function is linked to the corresponding channel read request queue
	struct sub_request *subs_w_tail;

	struct event_node *event;            //Event queue,add to the queue in chronological order, and at the end of the simulate function, the time is determined based on the time of the queue queue
	struct channel_info *channel_head;   //Points to the first address of the channel structure array
};


struct channel_info{
	int chip;                            //Indicates how many particles are on the bus
	unsigned int token;                  //In the dynamic allocation, in order to prevent each assignment in the first chip need to maintain a token, each time from the token referred to the location of the distribution

	int current_state;                   //channel has serveral states, including idle, command/address transfer,data transfer,unknown
	int next_state;
	__int64 current_time;                //Record the current time of the channel
	__int64 next_state_predict_time;     //the predict time of next state, used to decide the sate at the moment

	struct event_node *event;
	struct sub_request *subs_r_head;     //The read request on the channel queue header, the first service in the queue header request
	struct sub_request *subs_r_tail;     //Channel on the read request queue tail, the new sub-request added to the tail
	struct sub_request *subs_w_head;     //The write request on the channel queue header, the first service in the queue header request
	struct sub_request *subs_w_tail;     //The write request queue on the channel, the new incoming request is added to the end of the queue
	struct gc_operation *gc_command;     //Record the need to generate gc position

	unsigned long channel_read_count;	 //Record the number of read and write wipes within the channel
	unsigned long channel_program_count;
	unsigned long channel_erase_count;

	struct chip_info *chip_head;        
};


struct chip_info{
	unsigned int die_num;               //Indicates how many die is in a chip
	unsigned int plane_num_die;         //indicate how many planes in a die
	unsigned int block_num_plane;       //indicate how many blocks in a plane
	unsigned int page_num_block;        //indicate how many pages in a block
	unsigned int subpage_num_page;      //indicate how many subpage in a page
	unsigned int ers_limit;             //The number of times each block in the chip can be erased
	unsigned int token;                 //In the dynamic allocation, in order to prevent each assignment in the first die need to maintain a token, each time from the token referred to the location of the distribution
	
	int current_state;                  //chip has serveral states, including idle, command/address transfer,data transfer,unknown
	int next_state;
	__int64 current_time;               //Record the current time of the chip
	__int64 next_state_predict_time;    //the predict time of next state, used to decide the sate at the moment
 
	unsigned long chip_read_count;      //Record the number of read/program/erase in the chip
	unsigned long chip_program_count;
	unsigned long chip_erase_count;

    struct ac_time_characteristics ac_timing;  
	struct die_info *die_head;
};


struct die_info{

	unsigned int token;                 //In the dynamic allocation, in order to prevent each assignment in the first die need to maintain a token, each time from the token referred to the location of the distribution

	unsigned long die_read_count;		//Record the number of read/program/erase in the die
	unsigned long die_program_count;
	unsigned long die_erase_count;

	struct plane_info *plane_head;
	
};


struct plane_info{
	int add_reg_ppn;                    //Read, write address to the variable, the variable represents the address register. When the die is changed from busy to idle, clear the address
	unsigned int free_page;             //the number of free page in plane
	unsigned int ers_invalid;           //Record the number of blocks in the plane that are erased
	unsigned int active_block;          //The physical block number of the active block
	int can_erase_block;                //Record in a plane prepared in the gc operation was erased block, -1 that has not found a suitable block

	unsigned long plane_read_count;		//Record the number of read/program/erase in the plane
	unsigned long plane_program_count;
	unsigned long plane_erase_count;
	unsigned long pre_plane_write_count;

	struct direct_erase *erase_node;    //Used to record can be directly deleted block number, access to the new ppn, whenever the invalid_page_num == 64, it will be added to the pointer, for the GC operation directly delete
	struct blk_info *blk_head;
};


struct blk_info{
	unsigned int erase_count;          //The number of erasures for the block, which is recorded in ram for the GC
	unsigned int page_read_count;	   //Record the number of read pages of the block
	unsigned int page_write_count;	   //Record the number of write pages
	unsigned int pre_write_count;	   //Record the number of times the prepress was written

	unsigned int free_page_num;        //Record the number of pages in the block
	unsigned int invalid_page_num;     //Record the number of invaild pages in the block
	int last_write_page;               //Records the number of pages executed by the last write operation, and -1 indicates that no page has been written
	struct page_info *page_head;       
};


struct page_info{                      //lpn records the physical page stored in the logical page, when the logical page is valid, valid_state>0, free_state>0
	int valid_state;                   //indicate the page is valid or invalid
	int free_state;                    //each bit indicates the subpage is free or occupted. 1 indicates that the bit is free and 0 indicates that the bit is used
	unsigned int lpn;                 
	unsigned int written_count;        //Record the number of times the page was written
};


struct dram_info{
	unsigned int dram_capacity;     
	__int64 current_time;

	struct dram_parameter *dram_paramters;      
	struct map_info *map;
	struct buffer_info *buffer; 

};


/*****************************************************************************************************************************************
*Buff strategy:Blocking buff strategy
*1--first check the buffer is full, if dissatisfied, check whether the current request to put down the data, if so, put the current request, 
*if not, then block the buffer;
*
*2--If buffer is blocked, select the replacement of the two ends of the page. If the two full page, then issued together to lift the buffer 
*block; if a partial page 1 full page or 2 partial page, then issued a pre-read request, waiting for the completion of full page and then issued 
*And then release the buffer block.
********************************************************************************************************************************************/
typedef struct buffer_group{
	TREE_NODE node;                     //The structure of the tree node must be placed at the top of the user-defined structure
	struct buffer_group *LRU_link_next;	// next node in LRU list
	struct buffer_group *LRU_link_pre;	// previous node in LRU list

	unsigned int group;                 //the first data logic sector number of a group stored in buffer 
	unsigned int stored;                //indicate the sector is stored in buffer or not. 1 indicates the sector is stored and 0 indicate the sector isn't stored.EX.  00110011 indicates the first, second, fifth, sixth sector is stored in buffer.
	unsigned int dirty_clean;           //it is flag of the data has been modified, one bit indicates one subpage. EX. 0001 indicates the first subpage is dirty
	int flag;			                //indicates if this node is the last 20% of the LRU list	
	unsigned int page_type;				//buff page type:0--full_page  1--partial_page
}buf_node;


struct dram_parameter{
	float active_current;
	float sleep_current;
	float voltage;
	int clock_time;
};


struct map_info{
	struct entry *map_entry;            // each entry indicate a mapping information
	struct buffer_info *attach_info;	// info about attach map
};


struct controller_info{
	unsigned int frequency;             //Indicates the operating frequency of the controller
	__int64 clock_time;                 //Indicates the time of a clock cycle
	float power;                        //Indicates the energy consumption per unit time of the controller
};


struct request{
	__int64 time;                      //Request to reach the time(us)
	unsigned int lsn;                  //The starting address of the request, the logical address
	unsigned int size;                 //The size of the request, the number of sectors
	unsigned int operation;            //The type of request, 1 for the read, 0 for the write
	unsigned int cmplt_flag;		   //Whether the request is executed, 0 means no execution, 1 means it has been executed

	unsigned int* need_distr_flag;
	unsigned int complete_lsn_count;   //record the count of lsn served by buffer

	int distri_flag;		           // indicate whether this request has been distributed already

	__int64 begin_time;
	__int64 response_time;
	double energy_consumption;         //Record the energy consumption of the request

	struct sub_request *subs;          //Record all sub-requests belonging to the request
	struct request *next_node;        
};


struct sub_request{
	unsigned int lpn;                  //The logical page number of the sub request
	unsigned int ppn;                  //The physical page number of the request
	unsigned int operation;            //Indicates the type of the sub request, except that read 1 write 0, there are erase, two plane and other operations
	int size;

	unsigned int current_state;        //Indicates the status of the subquery
	__int64 current_time;
	unsigned int next_state;
	__int64 next_state_predict_time;
	 unsigned int state;              //The requested sector status bit

	__int64 begin_time;               //Sub request start time
	__int64 complete_time;            //Record the processing time of the sub-request, the time that the data is actually written or read out

	struct local *location;           //In the static allocation and mixed allocation mode, it is known that lpn knows that the lpn is assigned to that channel, chip, die, plane, which is used to store the calculated address
	struct sub_request *next_subs;    //Points to the child request that belongs to the same request
	struct sub_request *next_node;    //Points to the next sub-request structure in the same channel
	struct sub_request *update;       //Update the write request, mount this pointer on

	unsigned int update_read_flag;   //Update the read flag

};


/***********************************************************************
*The event node controls the growth of time, and the increase in each time 
*is determined by the time of the most recent event
************************************************************************/
struct event_node{
	int type;                        //Record the type of the event, 1 for the command type, and 2 for the data transfer type
	__int64 predict_time;            //Record the expected time of this time to prevent the implementation of this time ahead of time
	struct event_node *next_node;
	struct event_node *pre_node;
};

struct parameter_value{
	unsigned int chip_num;          //the number of chip in ssd
	unsigned int dram_capacity;     //Record the DRAM capacity in SSD
	unsigned int cpu_sdram;         //sdram capacity in cpu

	unsigned int channel_number;    //Record the number of channels in the SSD, each channel is a separate bus
	unsigned int chip_channel[100]; //Set the number of channels in the SSD and the number of particles on each channel

	unsigned int die_chip;    
	unsigned int plane_die;
	unsigned int block_plane;
	unsigned int page_block;
	unsigned int subpage_page;

	unsigned int page_capacity;
	unsigned int subpage_capacity;


	unsigned int ers_limit;         //记录每个块可擦除的次数
	int address_mapping;            //记录映射的类型，1：page；2：block；3：fast
	int wear_leveling;              // WL算法
	int gc;                         //记录gc策略
	int clean_in_background;        //清除操作是否在前台完成
	int alloc_pool;                 //allocation pool 大小(plane，die，chip，channel),也就是拥有active_block的单位
	float overprovide;
	float gc_threshold;             //当达到这个阈值时，开始GC操作，在主动写策略中，开始GC操作后可以临时中断GC操作，服务新到的请求；在普通策略中，GC不可中断

	double operating_current;       //NAND FLASH的工作电流单位是uA
	double supply_voltage;	
	double dram_active_current;     //cpu sdram work current   uA
	double dram_standby_current;    //cpu sdram work current   uA
	double dram_refresh_current;    //cpu sdram work current   uA
	double dram_voltage;            //cpu sdram work voltage  V

	int buffer_management;          //indicates that there are buffer management or not
	int scheduling_algorithm;       //记录使用哪种调度算法，1:FCFS
	float quick_radio;
	int related_mapping;

	unsigned int time_step;
	unsigned int small_large_write; //the threshould of large write, large write do not occupt buffer, which is written back to flash directly

	int striping;                   //表示是否使用了striping方式，0表示没有，1表示有
	int interleaving;
	int pipelining;
	int threshold_fixed_adjust;
	int threshold_value;
	int active_write;               //表示是否执行主动写操作1,yes;0,no
	float gc_hard_threshold;        //普通策略中用不到该参数，只有在主动写策略中，当满足这个阈值时，GC操作不可中断
	int allocation_scheme;          //记录分配方式的选择，0表示动态分配，1表示静态分配
	int static_allocation;          //记录是那种静态分配方式，如ICS09那篇文章所述的所有静态分配方式
	int dynamic_allocation;         //记录动态分配的方式
	int dynamic_allocation_priority; //动态分配的 优先级
	int advanced_commands;  
	int ad_priority;                //record the priority between two plane operation and interleave operation
	int ad_priority2;               //record the priority of channel-level, 0 indicates that the priority order of channel-level is highest; 1 indicates the contrary
	int greed_CB_ad;                //0 don't use copyback advanced commands greedily; 1 use copyback advanced commands greedily
	int greed_MPW_ad;               //0 don't use multi-plane write advanced commands greedily; 1 use multi-plane write advanced commands greedily
	int aged;                       //1表示需要将这个SSD变成aged，0表示需要将这个SSD保持non-aged
	float aged_ratio; 
	int queue_length;               //请求队列的长度限制

	struct ac_time_characteristics time_characteristics;
};

/********************************************************
*mapping information,state的最高位表示是否有附加映射关系
*********************************************************/
struct entry{                       
	unsigned int pn;                //物理号，既可以表示物理页号，也可以表示物理子页号，也可以表示物理块号
	int state;                      //十六进制表示的话是0000-FFFF，每位表示相应的子页是否有效（页映射）。比如在这个页中，0，1号子页有效，2，3无效，这个应该是0x0003.
};


struct local{          
	unsigned int channel;
	unsigned int chip;
	unsigned int die;
	unsigned int plane;
	unsigned int block;
	unsigned int page;
	unsigned int sub_page;
};


struct gc_info{
	__int64 begin_time;            //记录一个plane什么时候开始gc操作的
	int copy_back_count;    
	int erase_count;
	__int64 process_time;          //该plane花了多少时间在gc操作上
	double energy_consumption;     //该plane花了多少能量在gc操作上
};


struct direct_erase{
	unsigned int block;
	struct direct_erase *next_node;
};


/**************************************************************************************
 *当产生一个GC操作时，将这个结构挂在相应的channel上，等待channel空闲时，发出GC操作命令
***************************************************************************************/
struct gc_operation{          
	unsigned int chip;
	unsigned int die;
	unsigned int plane[2];
	unsigned int block;           //该参数只在可中断的gc函数中使用（gc_interrupt），用来记录已近找出来的目标块号
	unsigned int page;            //该参数只在可中断的gc函数中使用（gc_interrupt），用来记录已经完成的数据迁移的页号
	unsigned int state;           //记录当前gc请求的状态
	unsigned int priority;        //记录该gc操作的优先级，1表示不可中断，0表示可中断（软阈值产生的gc请求）
	struct gc_operation *next_node;
};

/*
*add by ninja
*used for map_pre function
*/
typedef struct Dram_write_map
{
	unsigned int state; 
}Dram_write_map;


struct ssd_info *initiation(struct ssd_info *);
struct parameter_value *load_parameters(char parameter_file[30]);
struct page_info * initialize_page(struct page_info * p_page);
struct blk_info * initialize_block(struct blk_info * p_block,struct parameter_value *parameter);
struct plane_info * initialize_plane(struct plane_info * p_plane,struct parameter_value *parameter );
struct die_info * initialize_die(struct die_info * p_die,struct parameter_value *parameter,long long current_time );
struct chip_info * initialize_chip(struct chip_info * p_chip,struct parameter_value *parameter,long long current_time );
struct ssd_info * initialize_channels(struct ssd_info * ssd );
struct dram_info * initialize_dram(struct ssd_info * ssd);
void initialize_statistic(struct ssd_info * ssd);