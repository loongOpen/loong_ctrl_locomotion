Copyright 2025 国地共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net/

# 简介

OpenLoong framework中locomotion主控制算法组件

# 使用说明

所有常规操作（编译、运行）在tools内通过脚本完成！

不需要自己命令行编译！

# 框架核心思想：隔离！

为防止代码可读性变差，任何添加的功能模块，必须以最小成本、最小作用域的形式进行添加。

# 框架结构

框架顶层为状态机fsm

添加功能的最大单位被称为plan，plan被add进状态机fsm，fsm负责调度所有的plan

plan内完成各自所需的所有子功能模块，不同plan之间在调度上互相独立。

plan之间复用的模块、数据，推荐以单例形式调用

程序初始会调用每个plan的init

程序运行过程中fms会调用plan的进入(hey)、运行(run)、通知退出(bye)、写日志(log)四个接口，plan自身通过运行(run)的返回值负责自身的退出。另外fms会根据plan的优先级强制plan退出
