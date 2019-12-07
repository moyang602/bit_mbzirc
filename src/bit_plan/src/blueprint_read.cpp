//此文件要写一个类BrickPlan，BrickPlan需要有一个函数get_next_brick()返回下一块砖的磁铁应该在空间的哪个位置
//问题1：前提要保证墙体都是完整的矩形


#include "blueprint_read.h"

BrickPlan::BrickPlan()
{
	fir_wall_i = fir_wall_j = 0;
	sec_wall_i = sec_wall_j = 0;

	//用来测试的墙体
	vector< vector<int> > fir_tempt{vector<int>{18,18,18, 18,18,18, 18,18,18, 18,18,18},
								    vector<int>{18,18,18, 18,18,18, 18,18,18, 18,18,18},
								    vector<int>{18,18,18, 18,18,18, 18,18,18, 18,18,18}
	};
	vector< vector<int> > sec_tempt{vector<int>{3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3},
								    vector<int>{6,6, 6,6, 6,6, 6,6, 6,6, 6,6},
							        vector<int>{12,12,12,12, 12,12,12,12, 12,12,12,12}
	};
	fir_wall = fir_tempt;
	sec_wall = sec_tempt;
}

BrickPlan::~BrickPlan()
{

}

//return: vector<double> 分别为xyz
vector<double> BrickPlan::get_fir_wall_nextbrick_xyz()
{
	if (fir_wall_j >= fir_wall.size())
	{
		return vector<double>();//超出墙的高度
	}
	int current_brick = fir_wall[fir_wall_j][fir_wall_i];//记录当前砖块种类，取值3,6,12,18
	int pre_i = fir_wall_i, pre_j = fir_wall_j;//记录起始的ij
	int occupied_grids = current_brick;//当前这块砖占的格子数(一个格子1dm，3的砖块就要占3个格子）
	//指针移动到下一块砖
	for (int i = 0; i < occupied_grids; i++)
	{
		fir_wall_i++;//宽度优先弹出砖块
	}
	if (fir_wall_i >= fir_wall[0].size())//宽度方向到头了
	{
		fir_wall_i = 0;
		fir_wall_j+=2;
	}
	//计算当前处理的砖的磁铁中心位置,单位为dm
	double x = pre_i * 1.0 + occupied_grids / 2.0*1.0;//单位dm，x等于前面所有格子的长度+半个砖块的长度
	double y = 0;
	double z = pre_j * 1.0 + 2.0;//z等于高度方向所有砖块的高度
	vector<double> res{ 100*x,100*y,100*z };//转换单位为mm
	return res;
}
//return: vector<double> 分别为xyz
vector<double> BrickPlan::get_sec_wall_nextbrick_xyz()
{
	if (sec_wall_j >= sec_wall.size())
	{
		return vector<double>();//超出墙的高度
	}
	int current_brick = sec_wall[sec_wall_j][sec_wall_i];//记录当前砖块种类，取值3,6,12,18
	int pre_i = sec_wall_i, pre_j = sec_wall_j;//记录当前砖块起始的ij
	int occupied_grids = current_brick;//当前这块砖占的格子数(一个格子1dm，3的砖块就要占3个格子）
	//指针移动到下一块砖
	for (int i = 0; i < occupied_grids; i++)
	{
		sec_wall_i++;//宽度优先弹出砖块
	}
	if (sec_wall_i >= sec_wall[0].size())//宽度方向到头了
	{
		sec_wall_i = 0;
		sec_wall_j+=2;
	}
	//计算当前处理的砖的磁铁中心位置,单位为dm
	double x = 1;//原点在fir_wall的端面上
	double y =1 + pre_i * 1.0 + occupied_grids / 2.0*1.0;//单位dm，x等于sec_wall起点偏差的0.5+前面所有格子的长度+半个砖块的长度
	double z = pre_j * 1.0 + 2.0;//z等于高度方向所有格子的长度
	vector<double> res{ 100*x,100*y,100*z };//转换单位到mm
	return res;
}

bool BrickPlan::fir_wall_isFinished()
{
	//z方向高度大于墙体高度，则这面墙处理完毕
	if (fir_wall_j >= fir_wall.size()) {
		return true;
	}
	else
	{
		return false;
	}
}

bool BrickPlan::sec_wall_isFinished()
{
	if (sec_wall_j>=sec_wall.size())
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
int main()
{
	BrickPlan brickplan;
	while (!brickplan.sec_wall_isFinished())
	{
		vector<double> cor = brickplan.get_sec_wall_nextbrick_xyz();
		for (size_t i = 0; i < cor.size(); i++)
		{
			cout << cor[i] << "  ";
		}
		cout << "\n";

	}
    std::cout << "Hello World!\n"; 
}
*/
