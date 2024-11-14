# Brief

此仓库仅作为本人在RoboMaster创梦之翼的工作记录，包含能量机关的识别和预测，本套代码理论上应在创梦之翼自瞄的基础上运行，但由于自瞄代码属于战队内部资源，不应该在个人仓库中公开，因此本套代码不保证能运行，仅作记录。

<h1 id="ZSUxY">整体逻辑</h1>

> **<font style="color:#DF2A3F;">务必认真阅读完规则手册后再阅读以下内容</font>**

<h2 id="k7JUY">架构</h2>

> 不含自瞄

![画板](https://cdn.nlark.com/yuque/0/2024/jpeg/38870365/1723084014378-22c4591f-0045-44a0-8496-b4dfe5860bfe.jpeg)

<h2 id="xyb3w">代码中的约定</h2>

1. 扇叶：代码中描述扇叶的变量名一般为`fanblade`，如下图蓝色框框选处，共五片。

![](https://cdn.nlark.com/yuque/0/2024/png/38870365/1723009387198-5182fb63-3485-4f57-88ea-e2d2bba21de7.png)

2. R 型标：代码中描述R型标的变量名一般为`R`，如下图，一般位于能量机关装置的中心凸起处。

![](https://cdn.nlark.com/yuque/0/2024/png/38870365/1722999853779-b2c511be-2e26-4140-95d8-4af39e148389.png)

3. 流水灯：代码中描述流水灯的结构体名为`WaterfallLight`，如下图黄色箭头所指处。

![](https://cdn.nlark.com/yuque/0/2024/png/38870365/1723009275311-5b4b24b7-73b9-4de0-9259-4a94f71efbf4.png)

4. 月牙形框：代码中描述月牙形框架的结构体名为`MoonFrame`，如下图蓝色框圈出的区域所示；其中月牙形框又分为`outside_moon`和`inside_moon`，顾名思义外月牙就是距离 R 标远的，内月牙就是距离 R 标近的，下图所示上蓝框为外月牙、下蓝框为内月牙。**（注：历史原因，在调试图像上，Moon 即为 outside_moon，T 即为 inside_moon）**

![](https://cdn.nlark.com/yuque/0/2024/png/38870365/1723009195134-f27d583e-211a-436c-b31a-e22c0eeec167.png)

<h2 id="tc5TZ">基本信息</h2>
<h3 id="XGiWs">坐标系定义</h3>

[坐标系定义](https://heurm.yuque.com/pwxb48/yy4glk/bslnoq1kqgctcn8u)

<h3 id="Dmo4t">自定义消息</h3>
<details class="lake-collapse"><summary id="u303aabe0"><span class="ne-text">Fanblade.msg</span></summary><p id="u639bec66" class="ne-p"><span class="ne-text">std_msgs/Header header</span></p><p id="u19be80c9" class="ne-p"><span class="ne-text">geometry_msgs/Pose fanblade_center		//扇叶中心坐标</span></p><p id="u564ff9aa" class="ne-p"><span class="ne-text">geometry_msgs/Pose r_center   			//能量机关中心R型标坐标</span></p><p id="u534cded6" class="ne-p"><span class="ne-text">geometry_msgs/Pose quaternion</span></p><p id="ua579ef77" class="ne-p"><span class="ne-text">float64[3] t_vec</span></p><p id="u2b256359" class="ne-p"><span class="ne-text">uint8 rotation 0						//能量机关转向，默认为零，1顺2逆</span></p></details>
<details class="lake-collapse"><summary id="u9b0e4747"><span class="ne-text">Target.msg</span></summary><p id="u482fdbef" class="ne-p"><span class="ne-text">std_msgs/Header header</span></p><p id="udaeb2300" class="ne-p"><span class="ne-text">geometry_msgs/Pose pose				//待击打目标点位姿</span></p><p id="u6df937f8" class="ne-p"><span class="ne-text">float64 angle						//当前扇叶角度</span></p><p id="uead69464" class="ne-p"><span class="ne-text">bool is_tracked</span></p></details>
<details class="lake-collapse"><summary id="uba20259f"><span class="ne-text">TrackedRune.msg</span></summary><p id="uf0e9416a" class="ne-p"><span class="ne-text">std_msgs/Header header</span></p><p id="u5c7ea6f0" class="ne-p"><span class="ne-text">TargetState target_state</span></p><p id="u8b2c2929" class="ne-p"><span class="ne-text">bool is_tracked</span></p></details>
<h3 id="Tcfbh">各节点职能</h3>

+ `recognition-node`：接受从相机节点中发来的`/camera/front/capture`、`/camera/front/camera_info`进行图像处理识别，经过一系列函数`preprocess`、`findWaterfallLight`、`findMoonFrame`、`matchFanBlade`、`solvepnp`，构建 `Fanblade`消息以 `/rune/fanblade`发送出去
+ `transform-node`：将接收到的相机系下的坐标转换为惯性系下
+ `prediction-node`：进行大小符预测，解算出惯性系下待击打目标点，构建 `Target`消息发送出去
+ `tracker-node`：将待击打的目标点套一层匀速卡尔曼滤波，使其更加平滑，减少云台抖动。**该节点滤波器部分主要是**[@花譜](undefined/huapu-zxjcq)**所写**。

<h2 id="dXFCQ">识别逻辑</h2>

> 只做概述，细节见代码

整体识别逻辑代码中较为清晰，此处不做赘述，但有几处判断逻辑需要额外说明一下：

1. 颜色判断：取流水灯区域为 ROI，进行通道分离，**根据红蓝通道均值大小判断颜色**，红色通道均值大于蓝的为红，反之为蓝，详见代码；
2. 内外月牙框的判断：遍历所有月牙形框，取与流水灯在一条直线上的两个框，距离流水灯近者为内月牙框，远的则为外月牙形框，详见代码；
3. 能量机关转向的判断：记录前后两帧内外月牙框中心坐标，记内月牙框中心到外月牙框中心的向量为`v`，利用前后两帧**向量的叉乘的正负**判断顺逆时针旋转，详见代码；
4. pnp 取点逻辑：计算方法使用`cv::SOLVEPNP_IPPE`，图像上的四点位置及顺序见下图。具体的选点方法也是根据**向量叉乘结果的正负号**判断的，代码已足够清楚，这里不再赘述。

![](https://cdn.nlark.com/yuque/0/2024/png/38870365/1723008446802-c4ac23d2-6b4f-4c3b-a6cf-4e1f88329490.png)![](https://cdn.nlark.com/yuque/0/2024/png/38870365/1723008552726-027d9fca-c07d-457d-a52a-4c0188090048.png)

需要注意的是识别使用了**两套参数**：`reco_config.yaml`和`reco_home_test.yaml`，前者为赛场光照环境下识别官方能量机关的参数，后者为家里场地光照环境下识别参数，在`launch`中可以切换。

<h2 id="BVY5I">预测逻辑</h2>

> 只做概述，细节见代码

预测部分除模型外唯一需要说明的就是模型求解之前的数据处理，由于两帧之间的时间差过于小，直接算角速度就会有较大的误差，因为这里的时间数量级很小，一个微小的误差会被无限放大，导致解算出的角速度误差很大，为避免这种情况，代码中直接使用**角度**和**时间**这两个自变量求解模型。代码中的数据处理部分目标在于创建一个从零开始的、具有目标长度的序列，利用这两个序列求解模型。代码已足够明了。

对于模型部分，代码实现了GD（梯度下降法）、[Ceres](http://ceres-solver.org/)（google的<font style="color:rgb(64, 64, 64);background-color:rgb(252, 252, 252);">一个开源 C++ 库，用于建模和解决大型、复杂的优化问题</font>）以及[高斯牛顿法](https://github.com/SnocrashWang/RuneLab)。具体步骤可参考链接，不过实测下来Ceres和高斯牛顿法效果不佳，建议换更好的模型，可以参考一下这个[GitHub - FaterYU/rm_buff: RoboMaster buff detect and predict](https://github.com/FaterYU/rm_buff)，感觉不错。

<h1 id="x8JGo">识别调试方法</h1>

识别添加了在调试图像中的错误参数显示，在轮廓面积参数通过的情况下，对于错误识别的物体会以**白色**将其轮廓画出，并将参数错误值标注在轮廓旁边，具体如下图所示（这里显示的是未通过的`MoonFrame`的`asp`参数实际值，画出的白色轮廓有些不明显，但放大还是可以看到的），然后根据显示的实际值调整相关参数，参考下面的调试视频（软件为 [Foxglove](https://foxglove.dev/)，可以与ROS2联动，实现实时调参、实时更改）。

![](https://cdn.nlark.com/yuque/0/2024/png/38870365/1723082470147-76a6d725-4c06-4b83-93eb-2e0768c8447a.png)

<h1 id="zNVFX">目前存在问题</h1>

1. 预测模型收敛准确度不高，赛场上临时改用了一种极为狗屎的方法，详见代码。建议直接换模型；
2. `tracker`节点匀速模型滤波有问题：滤波出的坐标与实际坐标有微小偏移，如图，这导致实际击打时需要给予 yaw, pitch 一定量的偏移，在自瞄`firecontrol`节点中专门加了关于能量机关的偏移补偿。

![](https://cdn.nlark.com/yuque/0/2024/png/38870365/1723011642604-4afcf32a-3c81-4cbb-ad3a-fa3f84cddc0e.png)

<h1 id="e6cX2">未来改进方向</h1>

1. 预测模型改进：高斯牛顿法和梯度下降法收敛速度较慢且预测准确度不高，可以尝试对能量机关使用拓展卡尔曼滤波进行整体建模，参考[GitHub - FaterYU/rm_buff: RoboMaster buff detect and predict](https://github.com/FaterYU/rm_buff)；
2. `tracker`节点滤波器改进：目前`tracker`中采用的匀速模型卡尔曼滤波不太可靠，需要探索出更适合能量机关的、专用的模型；
3. 自动击打：目前小符的命中率接近100%，可以尝试改用自动击打。大符在保证手动击打具有较高准确率的情况下，也可以尝试自动击打；
4. 神经网络识别：预测准确度较高、击打效果较好的情况下，可以考虑将识别换成神经网络。
5. **<font style="color:#E4495B;">一些建议：强烈建议优先优化预测模型和滤波器。如果官方不对能量机关整体形状做大的改变的话，识别可以不用改动，把更多的时间放在预测模型的优化上；在保证预测准确度较高的情况下再尝试神经网络识别。</font>**

<h1 id="BK7XE">可用资源</h1>

+ 调试视频也已上传至语雀视觉组资源处，在 ROSbag 文件夹中
