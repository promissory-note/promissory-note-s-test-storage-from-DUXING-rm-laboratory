#基于“二五”计划各项子计划，现正式刻写本人第一个伪代码

#项目简介：写了个字符输出程序，运行预期输出“hello robomaster";
#环境：
- 操作系统： Ubuntu
-编译器： gcc(支持c++17)
-构建工具：CMake （版本3.10 及以上—）
##构建命令
打开终端，在项目在项目根目录下依次执行以下命令：
’‘’bash
cmake -S . -B build
cmake --build build
-项目文件夹：pro2.cpp
-项目名：pro1.cpp

-目录结构说明：
项目根目录：
hello_cmake/

-存放代码文件夹： workplace/

-c++代码：pro1.cpp

- 构建CMake编译说明： CMakeLists.txt

- 说明文档：README.md

-告诉git哪些文件不要上传： 。gitignore