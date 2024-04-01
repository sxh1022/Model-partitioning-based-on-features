# Model-partitioning-based-on-features

## PART 1

```
基于模型特征的分块
```

![模型分块流程](F:\github\model-partition\images\模型分块流程.png)

![image-20230920102511866](images/image-20230920102511866.png)

- 按照模型特征进行分块

![image-20230912102533599](images/image-20230912102533599.png)

## PART 2

```
基于OBB包围盒的均匀分块 与 基于曲率的模型分块
```

- 长边方向均匀分块

![image-20230913133754623](images/image-20230913133754623.png)

- 按照曲率分块

![image-20230928160740923](images/image-20230928160740923.png)

## PART 3

```
将上述两种分块方法结合起来，同时要考虑小分区的合并
```

![image-20230928143843833](F:\github\model-partition\images\image-20230928143843833.png)