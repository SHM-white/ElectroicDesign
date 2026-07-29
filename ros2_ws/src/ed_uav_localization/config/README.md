# 仿真场地配置

`fields/simulation_arena.yaml` 是用于仿真器冒烟运行的合成几何数据。
它不是实测硬件场地数据，其来源明确为 `synthetic_simulation`，且设置为 `activation: blocked`。
任务执行器只有在使用 `simulation_only:=true` 启动时才接受它；它不能启用竞赛或硬件飞行。
