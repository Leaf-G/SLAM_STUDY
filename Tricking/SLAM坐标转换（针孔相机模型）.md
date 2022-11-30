# SLAM坐标转换（针孔相机模型）
## 路线
世界坐标->相机坐标->图像物理坐标->像素坐标
## 世界坐标->相机坐标
针对于点P，从其世界坐标转换到相机坐标，P点不发生位姿变化，两坐标仅为坐标系不同而产生不同的坐标值。
所以从P点的世界坐标到相机坐标可以认为是将坐标系进行了一次位姿变换，而对于当前P点的相机坐标系而言它的**世界坐标系**即为进行**位姿变换前的上一个相机坐标系**，所以对于P点而言，它的相机坐标Pc与世界坐标的关系如下：即一个变换矩阵T的问题。

$$
\begin{pmatrix}  
  X_{c} \\  
  Y_{c} \\
  Z_{c} \\
  1
\end{pmatrix} =
\begin{pmatrix}  
  R_{3 \times 3} & T_{3 \times 1} \\  
  o & 1  
\end{pmatrix} 
\begin{pmatrix}  
  X_{w} \\  
  Y_{w} \\
  Z_{w} \\
  1
\end{pmatrix} \tag{1}
$$
## 相机坐标->图像物理坐标
从相机坐标到图像物理坐标仅为一个成像原理，通过两个三角形相似可得：
$$
\left\{\begin{matrix} {} 
 X{}'=f\frac{X_{c}}{Z_{c}}\\
 Y{}'=f\frac{Y_{c}}{Z_{c}}   
\end{matrix}\right.\tag{2}
$$
## 图像物理坐标->像素坐标
图像物理坐标到像素坐标只需要烤炉对坐标的放缩和平移即：
$$
\left\{\begin{matrix} 
u=\alpha X{}' +c_{x}\\
v=\beta Y{}'+c_{y}  
\end{matrix}\right.\tag{3}
$$
## 投影
整合三个转换公式，并将α，f整合可得：
$$
Z\begin{pmatrix}  
  u \\  
  v\\
1  
\end{pmatrix} =
\begin{pmatrix}  
  f_{x}&0&c_{x} \\  
  0 &f_{y}&c_{y}\\
0&0&1  
\end{pmatrix}\begin{pmatrix}  
  X_{C} \\  
  Y_{C}\\
Z_{C}  
\end{pmatrix}
$$
