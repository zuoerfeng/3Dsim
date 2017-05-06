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
#define BUFSIZE 200

#define DYNAMIC_ALLOCATION 0
#define STATIC_ALLOCATION 1

#define INTERLEAVE 0
#define TWO_PLANE 1

#define NORMAL    2
#define INTERLEAVE_TWO_PLANE 3
#define COPY_BACK	4

#define AD_RANDOM 1     //并未使用
#define AD_COPYBACK 2
#define AD_TWOPLANE 4   //mutli plane write
#define AD_INTERLEAVE 8
#define AD_TWOPLANE_READ 16  //mutli plane read

#define READ 1
#define WRITE 0
#define UPDATE_READ 2

/*********************************all states of each objects************************************************
*一下定义了channel的空闲，命令地址传输，数据传输，传输，其他等状态
*还有chip的空闲，写忙，读忙，命令地址传输，数据传输，擦除忙，copyback忙，其他等状态
*还有读写子请求（sub）的等待，读命令地址传输，读，读数据传输，写命令地址传输，写数据传输，写传输，完成等状态
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

#define REQUEST_IN 300         //下一条请求到达的时间
#define OUTPUT 301             //下一次数据输出的时间

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

/*****************************************
*函数结果状态代码
*Status 是函数类型，其值是函数结果状态代码
******************************************/
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
	double ssd_energy;                   //SSD的能耗，是时间和芯片数的函数,能耗因子
	__int64 current_time;                //记录系统时间
	__int64 next_request_time;
	unsigned int real_time_subreq;       //记录实时的写请求个数，用在全动态分配时，channel优先的情况
	int flag;
	int active_flag;                     //记录主动写是否阻塞，如果发现柱塞，需要将时间向前推进,0表示没有阻塞，1表示被阻塞，需要向前推进时间
	unsigned int page;

	unsigned int token;                  //在动态分配中，为防止每次分配在第一个channel需要维持一个令牌，每次从令牌所指的位置开始分配
	unsigned int gc_request;             //记录在SSD中，当前时刻有多少gc操作的请求

	__int64 write_avg;                   //记录用于计算写请求平均响应时间的时间
	__int64 read_avg;                    //记录用于计算读请求平均响应时间的时间

	unsigned int min_lsn;
	unsigned int max_lsn;

	unsigned long read_count;
	unsigned long update_read_count;      //记录因为更新操作导致的额外读出操作
	unsigned long gc_read_count;			 //记录gc导致的读操作

	unsigned long program_count;
	unsigned long pre_all_write;		 //记录预处理写操作
	unsigned long update_write_count;	 //记录更新的写操作
	unsigned long gc_write_count;		 //记录gc导致的写操作

	unsigned long erase_count;
	unsigned long direct_erase_count;    //记录被直接擦除的无效块

	//记录高级命令执行读写操作
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

	unsigned long write_flash_count;     //实际产生的对flash的写操作
	unsigned long waste_page_count;      //记录因为高级命令的限制导致的页浪费

	unsigned long write_request_count;    //记录写操作的次数
	unsigned long read_request_count;     //记录读操作的次数
	
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

    struct parameter_value *parameter;   //SSD参数因子
	struct dram_info *dram;
	struct request *request_queue;       //dynamic request queue
	struct request *request_head;		 // the head of the request queue
	struct request *request_tail;	     // the tail of the request queue
	struct request *request_work;		 // the work point of the request queue

	struct sub_request *subs_w_head;     //当采用全动态分配时，分配是不知道应该挂载哪个channel上，所以先挂在ssd上，等进入process函数时才挂到相应的channel的读请求队列上
	struct sub_request *subs_w_tail;

	struct event_node *event;            //事件队列，每产生一个新的事件，按照时间顺序加到这个队列，在simulate函数最后，根据这个队列队首的时间，确定时间
	struct channel_info *channel_head;   //指向channel结构体数组的首地址
};


struct channel_info{
	int chip;                            //表示在该总线上有多少颗粒
	unsigned int token;                  //在动态分配中，为防止每次分配在第一个chip需要维持一个令牌，每次从令牌所指的位置开始分配

	int current_state;                   //channel has serveral states, including idle, command/address transfer,data transfer,unknown
	int next_state;
	__int64 current_time;                //记录该通道的当前时间
	__int64 next_state_predict_time;     //the predict time of next state, used to decide the sate at the moment

	struct event_node *event;
	struct sub_request *subs_r_head;     //channel上的读请求队列头，先服务处于队列头的子请求
	struct sub_request *subs_r_tail;     //channel上的读请求队列尾，新加进来的子请求加到队尾
	struct sub_request *subs_w_head;     //channel上的写请求队列头，先服务处于队列头的子请求
	struct sub_request *subs_w_tail;     //channel上的写请求队列，新加进来的子请求加到队尾
	struct gc_operation *gc_command;     //记录需要产生gc的位置

	unsigned long channel_read_count;	 //记录channel内的读写擦次数
	unsigned long channel_program_count;
	unsigned long channel_erase_count;

	struct chip_info *chip_head;        
};


struct chip_info{
	unsigned int die_num;               //表示一个颗粒中有多少个die
	unsigned int plane_num_die;         //indicate how many planes in a die
	unsigned int block_num_plane;       //indicate how many blocks in a plane
	unsigned int page_num_block;        //indicate how many pages in a block
	unsigned int subpage_num_page;      //indicate how many subpage in a page
	unsigned int ers_limit;             //该chip中每块能够被擦除的次数
	unsigned int token;                 //在动态分配中，为防止每次分配在第一个die需要维持一个令牌，每次从令牌所指的位置开始分配
	
	int current_state;                  //channel has serveral states, including idle, command/address transfer,data transfer,unknown
	int next_state;
	__int64 current_time;               //记录该通道的当前时间
	__int64 next_state_predict_time;    //the predict time of next state, used to decide the sate at the moment
 
	unsigned long chip_read_count;      //记录chip内的读写擦次数
	unsigned long chip_program_count;
	unsigned long chip_erase_count;

    struct ac_time_characteristics ac_timing;  
	struct die_info *die_head;
};


struct die_info{

	unsigned int token;                 //在动态分配中，为防止每次分配在第一个plane需要维持一个令牌，每次从令牌所指的位置开始分配

	unsigned long die_read_count;		//记录blk内的读写擦次数
	unsigned long die_program_count;
	unsigned long die_erase_count;

	struct plane_info *plane_head;
	
};


struct plane_info{
	int add_reg_ppn;                    //read，write时把地址传送到该变量，该变量代表地址寄存器。die由busy变为idle时，清除地址 //有可能因为一对多的映射，在一个读请求时，有多个相同的lpn，所以需要用ppn来区分  
	unsigned int free_page;             //该plane中有多少free page
	unsigned int ers_invalid;           //记录该plane中擦除失效的块数
	unsigned int active_block;          //if a die has a active block, 该项表示其物理块号
	int can_erase_block;                //记录在一个plane中准备在gc操作中被擦除操作的块,-1表示还没有找到合适的块

	unsigned long plane_read_count;		//记录plane内的读写擦次数
	unsigned long plane_program_count;
	unsigned long plane_erase_count;
	unsigned long pre_plane_write_count;

	struct direct_erase *erase_node;    //用来记录可以直接删除的块号,在获取新的ppn时，每当出现invalid_page_num==64时，将其添加到这个指针上，供GC操作时直接删除
	struct blk_info *blk_head;
};


struct blk_info{
	unsigned int erase_count;          //块的擦除次数，该项记录在ram中，用于GC
	unsigned int page_read_count;	   //记录块的read page次数	
	unsigned int page_write_count;	   //记录块的write page次数
	unsigned int pre_write_count;	   //记录预处理时候write page的次数

	unsigned int free_page_num;        //记录该块中的free页个数，同上
	unsigned int invalid_page_num;     //记录该块中失效页的个数，同上
	int last_write_page;               //记录最近一次写操作执行的页数,-1表示该块没有一页被写过
	struct page_info *page_head;       //记录每一子页的状态
};


struct page_info{                      //lpn记录该物理页存储的逻辑页，当该逻辑页有效时，valid_state大于0，free_state大于0；
	int valid_state;                   //indicate the page is valid or invalid
	int free_state;                    //each bit indicates the subpage is free or occupted. 1 indicates that the bit is free and 0 indicates that the bit is used
	unsigned int lpn;                 
	unsigned int written_count;        //记录该页被写的次数	  
};


struct dram_info{
	unsigned int dram_capacity;     
	__int64 current_time;

	struct dram_parameter *dram_paramters;      
	struct map_info *map;
	struct buffer_info *buffer; 

};


/*********************************************************************************************
*buffer中的已写回的页的管理方法:在buffer_info中维持一个队列:written。这个队列有队首，队尾。
*每次buffer management中，请求命中时，这个group要移到LRU的队首，同时看这个group中是否有已
*写回的lsn，有的话，需要将这个group同时移动到written队列的队尾。这个队列的增长和减少的方法
*如下:当需要通过删除已写回的lsn为新的写请求腾出空间时，在written队首中找出可以删除的lsn。
*当需要增加新的写回lsn时，找到可以写回的页，将这个group加到指针written_insert所指队列written
*节点前。我们需要再维持一个指针，在buffer的LRU队列中指向最老的一个写回了的页，下次要再写回时，
*只需由这个指针回退到前一个group写回即可。
**********************************************************************************************/
typedef struct buffer_group{
	TREE_NODE node;                     //树节点的结构一定要放在用户自定义结构的最前面，注意!
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
	struct entry *map_entry;            //该项是映射表结构体指针,each entry indicate a mapping information
	struct buffer_info *attach_info;	// info about attach map
};


struct controller_info{
	unsigned int frequency;             //表示该控制器的工作频率
	__int64 clock_time;                 //表示一个时钟周期的时间
	float power;                        //表示控制器单位时间的能耗
};


struct request{
	__int64 time;                      //请求到达的时间，单位为us,这里和通常的习惯不一样，通常的是ms为单位，这里需要有个单位变换过程
	unsigned int lsn;                  //请求的起始地址，逻辑地址
	unsigned int size;                 //请求的大小，既多少个扇区
	unsigned int operation;            //请求的种类，1为读，0为写
	unsigned int cmplt_flag;		   //请求是否被执行，0表示未执行，1表示已执行

	unsigned int* need_distr_flag;
	unsigned int complete_lsn_count;   //record the count of lsn served by buffer

	int distri_flag;		           // indicate whether this request has been distributed already

	__int64 begin_time;
	__int64 response_time;
	double energy_consumption;         //记录该请求的能量消耗，单位为uJ

	struct sub_request *subs;          //链接到属于该请求的所有子请求
	struct request *next_node;         //指向下一个请求结构体
};


struct sub_request{
	unsigned int lpn;                  //这里表示该子请求的逻辑页号
	unsigned int ppn;                  //分配那个物理子页给这个子请求。在multi_chip_page_mapping中，产生子页请求时可能就知道psn的值，其他时候psn的值由page_map_read,page_map_write等FTL最底层函数产生。 
	unsigned int operation;            //表示该子请求的类型，除了读1 写0，还有擦除，two plane等操作 
	int size;

	unsigned int current_state;        //表示该子请求所处的状态，见宏定义sub request
	__int64 current_time;
	unsigned int next_state;
	__int64 next_state_predict_time;
	 unsigned int state;              //使用state的最高位表示该子请求是否是一对多映射关系中的一个，是的话，需要读到buffer中。1表示是一对多，0表示不用写到buffer
	                                  //读请求不需要这个成员，lsn加size就可以分辨出该页的状态;但是写请求需要这个成员，大部分写子请求来自于buffer写回操作，可能有类似子页不连续的情况，所以需要单独维持该成员

	__int64 begin_time;               //子请求开始时间
	__int64 complete_time;            //记录该子请求的处理时间,既真正写入或者读出数据的时间

	struct local *location;           //在静态分配和混合分配方式中，已知lpn就知道该lpn该分配到那个channel，chip，die，plane，这个结构体用来保存计算得到的地址
	struct sub_request *next_subs;    //指向属于同一个request的子请求
	struct sub_request *next_node;    //指向同一个channel中下一个子请求结构体
	struct sub_request *update;       //因为在写操作中存在更新操作，因为在动态分配方式中无法使用copyback操作，需要将原来的页读出后才能进行写操作，所以，将因更新产生的读操作挂在这个指针上

	unsigned int update_read_flag;   //更新读完成的标志位

};


/***********************************************************************
*事件节点控制时间的增长，每次时间的增加是根据时间最近的一个事件来确定的
************************************************************************/
struct event_node{
	int type;                        //记录该事件的类型，1表示命令类型，2表示数据传输类型
	__int64 predict_time;            //记录这个时间开始的预计时间，防止提前执行这个时间
	struct event_node *next_node;
	struct event_node *pre_node;
};

struct parameter_value{
	unsigned int chip_num;          //记录一个SSD中有多少个颗粒
	unsigned int dram_capacity;     //记录SSD中DRAM capacity
	unsigned int cpu_sdram;         //记录片内有多少

	unsigned int channel_number;    //记录SSD中有多少个通道，每个通道是单独的bus
	unsigned int chip_channel[100]; //设置SSD中channel数和每channel上颗粒的数量

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
	unsigned int plane;
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