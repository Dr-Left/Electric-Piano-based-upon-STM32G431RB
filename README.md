# 基于STM32G431RB的简易电子琴的设计与实现

###### 左京伟  未央-电11  2021012328

## **一、** **实验设计**

### ***模式***：

简易电子琴包括**演奏**和**播放**两种模式，通过一定的讯号输入来切换。并且可以通过

####     演奏模式：可以弹奏围绕中央C的三个8度内的所有白键。

####     播放模式：可以播放4首曲目：

- 《清华大学校歌》

- 《明明就》-周杰伦

- 《十年》-陈奕迅

- 《Big Big World》- Emilia

同时可以作出播放/暂停、增减速度、升降调、升降八度、从头开始的控制操作。

 

### ***输出设备***：

有**直接控制蜂鸣器发声**和**作为****MIDI****设备控制电脑发声**两种输出模式。

 

### ***输入控制***：

远程、现场均可控制：可以通过扩展板上的8个**按键**以及外接的一个大按钮来控制，也可以用手机连接设备上的**蓝牙**通过串口传输控制消息。

 

![image](https://github.com/Dr-Left/Electric-Piano-based-upon-STM32G431RB/blob/master/1.png)



 

## **二、** **效果实现**

以上功能基本实现。

美中不足在于弹奏时，声音的长短不受键盘控制；并且键盘控制乐曲播放很容易误操作。限制因素有两个：一是键盘的抖动导致很难消除。二是通过查询方式查询键盘的状态过于消耗效率，使用上升、下降沿中断同时监测键盘的动作又过于繁琐。目前想到的办法是结合时钟信号，监测键盘连续*一定时长*（为了消抖）TTL电平处于低电平，但没有实现。

 

 

## **三、** 总结与思考

1. ### 调试遇到的“坎”

1) 一开始尝试在中断函数中调用HAL_Delay( )函数，结果程序直接跑飞。当时呈现的状况是，一旦调用了这个函数，板子的K1, K3还受控制，其他按键全部失灵。反复检查接线之后，我在出现异常状态时点击“暂停调试”按钮，然后一直点“Step Out”，发现点了几下之后绿色的高亮已经消失了，但调试仍未终止。我才意识到也许程序跑飞了。

> 后经老师指点：
>
> *HAL_Delay**函数（可以右键看到它的函数定义，实验课上分析过），以查询间方式，间接用到了**NVIC**中**system tick timer**中断，由这级中断提供**HAL_delay**的延时时间，**HAL_delay**的延时到了才往下执行。*
>
> *如果执行**HAL delay* *的外中断与**sys tick**中断同级，需要执行完外中断后，才能再执行**sys tick**中断。*
>
> *可是外中断里的**HAL delay* *需要等**systick**执行完指定的中断次数，即延时够了，才会往下执行。*
>
> *这样，造成外中断和**sys tick**中断互锁，你等我，我等你，一直卡在**HAL delay**函数的查询里。外中断一直不能往下执行，而**sys tick* *中断也一直无法响应，不被执行。*
>
> *借助这个互锁的例子，可以明白中断优先级设置不合理时的相互影响。*
>
> 明白原来还是自己太马虎[捂脸]
>

 

2) 刚实现增减8度的弹奏功能时，发现到高音区的Fa开始，声音听起来“跑调了”。后来在Debug模式下查看了变量的值，发现频率的计算并没有发生错误。后来反复思考，意识到高音区由于频率很高，我的时钟TIM3一开始的PSR值太大，导致最后输出PWM波的调节步长太长，在高音区误差尤其大。后来把PSR的值缩小了十倍，这个问题得到了很好地解决。

3) 刚实现蓝牙通讯的时候，发现单片机受到的手机发出的讯号全是0. 我一开始以为代码有问题，没有正确读取缓冲区，查了很久。后来用电脑的串口连接蓝牙模块，手机发送同样的信号，在串口调试助手上看收到的信号，也全部都是0. 我仔细想前几天的调试经验，突然想到也许是波特率不对。调节为9600之后发现问题解决了。

 

2. ### 上课得到的收获

  1) 对**计算机底层硬件**的工作了解更加深入了。以前学软件编程比较多，硬件基本是白痴状态。现在知道计算机硬件的基本架构，以及处理流程，并且我们应该如何去操作底层。

2) **Debug****的能力**又上了一个台阶。硬件的Debug和纯软件还不太一样。硬件的输出很不直观，不像在软件里面可以随时打印中间结果到console里面。这就对我们的Debug能力提出更高的要求。我们用到示波器等等一些间接的形式去反映硬件的工作状态，这是之前从来没有尝试过的。我们也试过把代码分部分注释，看哪一部分出了问题。

3) **做项目的综合能力**得到提升。这次大作业，无论是从功能的数目，代码量，还是硬件的针脚数目来说，都有一定的复杂度。这就需要我们一开始做好合理的规划。比如项目统共要实现哪些功能，两三天时间来不来得及调试完工，功能的可行性高不高，应用价值大不大；硬件的针脚怎么安排，不要到后面冲突起来，改起来很麻烦；代码的各个部分之间怎么连接，什么代码适合写成函数模块方便复用等等。这种综合统筹管理的能力只有通过这种大项目才能锻炼到。

 

> 代码见附件。
