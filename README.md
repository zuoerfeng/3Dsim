# 3Dsim
An SSD simulator for 3D flash, called 3Dsim.

the main function:

1.Support for 3D commands, for example:mutli plane\half page read\one shot read\ont shot program/Erase suspend/resume..etc

2.Multi-level parallel simulation,such as channel/chip/die/plane

3.Clear hierarchical interface

4.4-layer structure, include interface/buffer/ftl/fvl/flash level

3Dsim的一些运行问题及修改：
> 1.更改配置文件盘大小到512M,跑512M trace 出现盘写满的错误Error,原因：对盘做旧化处理之后，导致空闲页减少很多，小于trace的访问空间；
	3Dsim对盘做旧化处理方式，将无效页平均到每一个block中，导致大量块内遗留较多free~page，不符合ssd机制，ssd实际工作至多有一个块有空闲页。
	
    在修改后，增添pre_process_write()函数将上述大量空闲页置无效
	
> 2.代码中所有的chip~plane均采用的chip~plane[0]，即默认只用1个chip
	修改配置文件配置指定plane
	
> 3.通过跑financial2 trace，观察结果中mutli read结果一直为0
	代码中处理mutli read中存在指针传参未更改问题
    重新设置关于mutli read函数部分接口

> 4.发现当配置文件age=0,即不执行旧化函数的时候，添加的pre_process_write()会报错，修改如下，做个判断：
	if (ssd->parameter->aged == 1)
	{
		pre_process_write(ssd);   //将有效块中的free_page全部置为无效，保证最多一个有效块中包含有free page,满足实际ssd的机制
	}

> 5.根据设计需求，在动态分配中，将分配的优先级改为plane>channel>die

> 6.根据目前3D SSD的高级命令特性，更改所有的写命令，均按照mutli plane write执行

> 7.调整buffer策略，实现阻塞式buffer，保证每次从buffer从替换的写子请求均为两个，为高级命令mutli plane write服务，已修改完成，多个trace测试通过

> 8.更改gc策略，gc擦除无效块的粒度提升，提出superblock的特点，将一个die所有plane中block组成一个superblock，一次性擦除

> 9.支持高级命令half page read ,此高级命令只用于single plane的模式下

> 10.支持高级命令one shot program,此高级命令可以用于single/mutli plane的模式下

> 11.增加用于拼凑高级的命令的二级缓存command_buffer

> 12.增加高级命令one shot read,此高级命令可以用于single/mutli plane的模式下

> 13.对于读请求队列采用的FCFS，先来先服务的调度策略

> 14.增加高级命令Erase suspend/resume，此高级命令主要服务于gc操作中的erase

> 15.增加静态分配方式，在配置文件page.parameters中allocation中配置

> 16.增加基于负载的混合动态分配算法，可在配置文件中配置

> 17.在传统动态分配方式下，增加stripe写入方式，可在配置文件中配置

> 18.在传统动态分配方式下，增加one-shot-program-aware(OSPA)写入方式，可在配置文件中配置

> 19.增加基于负载的混合动态分配算法，负载感知通过静态表完成

> 20.更新关于3Dsim的用户手册说明文档 
.......


PS:上传github上对trace文件有所修改，源码能测试通过,需剔除github上trace，手动添加其他方式下载或生成trace即可
