# ssdsim
ssdsim的一些运行问题：
> 1.initialize.c文件的initiation（）函数可以修改trace文件名，和输出文件名。

    在修改后，有时候会出错，只要是strcpy_s函数的第二个参数，表示最大文件名，把这个参数改大一点就可以了
> 2.在跑程序的时候，会出现“can't find active block”的问题

    这个很可能是因为参数配置问题，在page.parameters文件中，overprovide参数要比gc hard threshold设置的大才行
> 3.在跑大trace的时候很可能会出现文件永远跑不完的情况

    这个问题需要修改ssd.c中的get_requests（）函数，这一份代码我已经改了，在函数前面添一段
      if(feof(ssd->tracefile)){
		    return 100; 
	    }
    就可以了
