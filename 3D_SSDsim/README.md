# ssdsim
ssdsim的一些运行问题及修改：
> 1.更改配置文件盘大小到512M,跑512M trace 出现盘写满的错误Error,原因：对盘做旧化处理之后，导致空闲页减少很多，小于trace的访问空间；
	SSDsim对盘做旧化处理方式，将无效页平均到每一个block中，导致大量块内遗留较多free~page，不符合ssd机制，ssd实际工作至多有一个块有空闲页。
	
    在修改后，增添pre_process_write()函数将上述大量空闲页置无效
	
> 2.：代码中所有的chip~plane均采用的chip~plane[0]，即默认只用1个chip
	修改配置文件配置指定plane
	
> 3.通过跑financial2 trace，观察结果中mutli read结果一直为0
	代码中处理mutli read中存在指针传参未更改问题
    重新设置关于mutli read函数部分接口

