#ifndef blueprint_read_h
#define blueprint_read_h

#include <iostream>
#include <vector>
using namespace std;

class BrickPlan
{
public:
	BrickPlan();
	~BrickPlan();
	//弹出沿x轴的墙的下一块砖
	vector<double> get_fir_wall_nextbrick_xyz();
	//弹出沿y轴的墙的下一块砖的位置
	vector<double> get_sec_wall_nextbrick_xyz();
	bool fir_wall_isFinished();//判断first wall是否完成
	bool sec_wall_isFinished();

private:
	//两个墙体，作为二维数组，每个元素的数值代表砖块长度，单位m；砖块横截面为0.2m*0.2m
	//元素取值有3、6、12、18，分别代表3dm、6dm等长度的砖块
	//墙体的角点在0点，wall_1延伸向x方向，wall_2延伸向y方向
	vector<vector<int>> fir_wall, sec_wall;	
	int fir_wall_column, fir_wall_row;//墙体wall_1输出的下一块转的起点位置，i指x/y方向，j指z方向,初始值应该为0
	int sec_wall_column, sec_wall_row;//同上

};

#endif

