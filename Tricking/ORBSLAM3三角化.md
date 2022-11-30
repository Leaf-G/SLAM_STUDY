# ORBSLAM3中的三角化
## 主要函数
在三角化中，ORBSLAM3直接使用前面已经完成了特征提取和特征匹配的结果，直接获取到特征点的像素坐标，由此，可以进行反投影从像素坐标得到相机坐标，再对相匹配的两个相机坐标进行三角化，所以这其中涉及到了两个主要函数，**反投影函数unproject与三角化函数Triangulate**。
*注：该函数定义在KannalaBrandt8相机模型文件中。*
## 反投影函数unproject
### 参数
该函数中参数只有一个待反投影的像素坐标点，const cv::Point2f引用类型。
### 内容
根据图像物理坐标的转换公式倒推即可得到反投影的方法：
$$
\left\{\begin{matrix} 
u=\alpha X{}' +c_{x}\\
v=\beta Y{}'+c_{y}  
\end{matrix}\right.\tag{1}
$$
其中$$\begin{pmatrix}  u \\  v   \end{pmatrix} $$是像素坐标，$$\begin{pmatrix}  
X_{}' \\  
Y_{}'   
\end{pmatrix} $$
是图像物理坐标。
另外，图像物理坐标与相机坐标的对应关系：
$$
\left\{\begin{matrix} {} 
 X{}'=f\frac{X}{Z}\\
 Y{}'=f\frac{Y}{Z}   
\end{matrix}\right.\tag{2}
$$
*注：由于在对特征点进行提取匹配时进行了归一化处理，所以得到的相机坐标开始，都是进行了归一化处理后的值，即对归一化坐标左乘内参即可得到像素坐标*
由(1)(2)可以推得反投影公式：
$$
\left\{\begin{matrix} {} 
 \frac{X}{Z}=\frac{u-c_{x}}{f_{x}}\\
 \frac{Y}{Z}=\frac{v-c_{y}}{f_{y}}   
\end{matrix}\right.\tag{3}
$$
得到归一化坐标。
反投影操作代码实现：
```cpp
cv::Point2f pw((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1]);
```
其中p2D表示需要进行反投影的二维坐标点，即对应归一化坐标，不考虑z=1的值。mvParameters容器包含四个相机内参，分别表示fx，fy，cx，cy。
此后在ORBSLAM3中还对失真进行迭代补偿
## 三角化函数Triangulat
### SLAM十四讲中的方法（利用叉乘进行消元计算）
$$
s_{2}x_{2}=s_{2}Rx_{1}+t \tag{4}
$$
两边同时叉乘x2，即同时左乘x2的反对称矩阵:
$$
s_{2}x_{2}^{\wedge } x_{2}=0=s_{2}x_{2}^{\wedge }Rx_{1}+x_{2}^{\wedge }t \tag{5}
$$
易得s1，s2的值。
### 参数
该方法中包含5个参数，分别为两个经过了重投影的归一化坐标，两个变换矩阵，一个保存P点的三维坐标。第一个变换矩阵为单位阵，表示以此为起点，第二个变换矩阵为从前一图片变换到后一图片的变换矩阵，通过特征提取特征匹配位姿估计得到。最后
两个归一化坐标参数为const cv::Point2f引用类型，两个矩阵为const Eigen::Matrix<float, 3, 4>引用类型，三维坐标点为：Eigen::Vector3f引用类型。
### 内容
ORBSLAM3中三角化采用的是线性三角化法。
$$世界坐标系中三维空间的其次坐标X=
\begin{bmatrix}  
  x\\
y\\
z\\
1
\end{bmatrix} 
，归一化坐标x=\begin{bmatrix}
p\\
q\\
1
\end{bmatrix}
,变换矩阵T=\begin{bmatrix}
r_{1}\\
r_{2}\\
r_{3}
\end{bmatrix}=\begin{bmatrix}
R&T
\end{bmatrix},
s为深度值
$$
对于某点的归一化坐标与其对应三维空间点在世界坐标系的坐标之间的对应关系：
$$
\begin{array}{l} 
 sx=TX\\
\Rightarrow sx\times TX=0\\
\Rightarrow x^{\wedge } TX=0
\end{array}  \tag{6}
$$
借助反对称矩阵可得：

$$
\begin{array}{c} 
x^{\wedge } TX=\begin{bmatrix}  
0&-1&q\\
1&0&-p\\
-q&p&0\\
\end{bmatrix} \begin{bmatrix}  
r1\\
r2\\
r3\\
\end{bmatrix} X\\=
\begin{bmatrix}  
-r_{2}+qr_{3}\\
r_{1}-pr_{3}\\
-qr_{2}+pr_{2}\\
\end{bmatrix}X  \\
\tag{7} 
\end{array}
$$
由于只有第一行和第二行是线性独立的，即保留前两行：
$$
\begin{array}{c} 
\begin{bmatrix}  
-r_{2}+qr_{3}\\
r_{1}-pr_{3}
\end{bmatrix}X  =0\tag{8} 
\end{array}$$
由于存在相匹配的两个点，由上式可得：
$$
\begin{array}{c} 
\begin{bmatrix}  
-r_{2}^{1}+q^{1}r_{3}^{1}\\
r_{1}^{1}-p^{1}r_{3}^{1}\\
-r_{2}^{2}+q^{2}r_{3}^{2}\\
r_{1}^{2}-p^{2}r_{3}^{2}
\end{bmatrix}X  =0,r^{1}r^{2}分别表示两个变换矩阵，p^{1},p^{2},q^{1},q^{2}分别表示两个匹配点的归一化坐标
\tag{9} 
\end{array}
$$
对应代码为：

```cpp
        Eigen::Matrix<float, 4, 4> A;
        A.row(0) = p1.x * Tcw1.row(2) - Tcw1.row(0);
        A.row(1) = p1.y * Tcw1.row(2) - Tcw1.row(1);
        A.row(2) = p2.x * Tcw2.row(2) - Tcw2.row(0);
        A.row(3) = p2.y * Tcw2.row(2) - Tcw2.row(1);
```
将上述结果组成一个4×4矩阵A。
上述方程没有非零解，使用SVD求得最小二乘解无法保证解满足齐次坐标，故对其进行归一化处理：
$$
X=\begin{bmatrix}  
x\\
y\\
z\\
1
\end{bmatrix} =\begin{bmatrix}  
x_{0}/x_{3}\\
x_{1}/x_{3}\\
x_{2}/x_{3}\\
x_{3}/x_{3}
\end{bmatrix}\tag{10}
$$
对应代码为：

```cpp
Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
        Eigen::Vector4f x3Dh = svd.matrixV().col(3);
        x3D = x3Dh.head(3) / x3Dh(3);
```
### main函数设计思路
通过对两张图片进行特征提取，特征匹配，位姿估计，从而得到从图片一到图片二的变换矩阵，对两张图片的特征点进行反投影，再讲参数传入三角化函数，即可完成三角化操作得到P的三维坐标点。
