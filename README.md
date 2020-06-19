# icp_learning
深蓝学院机器人学中的状态估计点云icp作业

分别用了SVD和G2O优化 两种方法。
```
git clone https://github.com/liangsongLee/icp_learning.git
cd WORK_PATH/icp_learning/3rdParty/g2o
mkdir build
cd build
cmake ..
make -j4
cd WORK_PATH/icp_learning
mkdir build
cd build
cmake ..
make -j4
```

Run in WORK_PATH/icp_learning
```
cd WORK_PATH/icp_learning
./build/icp_main
