//此文件要写一个类BrickPlan，BrickPlan需要有一个函数get_next_brick()返回下一块砖的磁铁应该在空间的哪个位置
//问题1：前提要保证墙体都是完整的矩形


#include "blueprint_read.h"

BrickPlan::BrickPlan()
{
	fir_wall_column = fir_wall_row = 0;
	sec_wall_column = sec_wall_row = 0;

	//用来测试的墙体
	//行数代表层数，列数=0表示离角点最远的那块砖
	vector<vector<int>> fir_tempt{ vector<int>{18, 18},
								   vector<int>{18, 18},
								   vector<int>{18, 18}
	};
	vector<vector<int>> sec_tempt{ vector<int>{3,3,3,3,3,3, 6,6,6},
								   vector<int>{12, 12, 12},
							       vector<int>{12, 6,6, 12}
	};
	fir_wall = fir_tempt;
	sec_wall = sec_tempt;
}

BrickPlan::~BrickPlan()
{

}
//return: vector<double> 分别为x,y,z,length，length为砖块长度，取值有0.3 0.6 1.2 1.8，请跟进长度判断砖块的种类
vector<double> BrickPlan::get_fir_wall_nextbrick_xyz()
{
	if (fir_wall_row >= fir_wall.size())
	{
		return vector<double>();//超出墙的高度
	}

	//先计算指针当前指的着块砖的磁铁中心位置,单位为m
	int current_brick = fir_wall[fir_wall_row][fir_wall_column];//记录当前砖块种类，取值3,6,12,18
	int cur_column = fir_wall_column, cur_row = fir_wall_row;//记录当前的column row
	double delta_x = 0; //砖块中心里墙外端的横向距离
	for (int i=0;i<fir_wall_column;i++)
	{
		delta_x += fir_wall[fir_wall_row][i] / 10.0; //分米转米
	}
	delta_x += fir_wall[fir_wall_row][fir_wall_column] / 10.0 / 2.0; //当前砖块的一般距离
	double x = 4.0 - delta_x;//当前砖块的中心的x坐标，墙体长4米
	double y = 0;
	double z = cur_row * 0.2 + 0.2;//z等于这层及以下所有砖块的高度,砖块高度为0.2m
	vector<double> res{ x,y,z, double(current_brick) / 10.0 };//转换单位为m

	//指针移动到下一块砖
	fir_wall_column++;
	if (fir_wall_column >= fir_wall[cur_row].size())//当前行宽度方向到头了
	{
		fir_wall_column = 0;
		fir_wall_row++;
	}
	return res;
}
//return: vector<double> 分别为xyz
vector<double> BrickPlan::get_sec_wall_nextbrick_xyz()
{
	if (sec_wall_row >= sec_wall.size())
	{
		return vector<double>();//超出墙的高度
	}

	//先计算指针当前指的着块砖的磁铁中心位置,单位为m
	int current_brick = sec_wall[sec_wall_row][sec_wall_column];//记录当前砖块种类，取值3,6,12,18
	int cur_column = sec_wall_column, cur_row = sec_wall_row;//记录当前的column row

	double x = 0.1;//
	double delta_y = 0; //砖块中心里墙外端的横向距离
	for (int i=0;i<sec_wall_column;i++)
	{
		delta_y += sec_wall[sec_wall_row][i] / 10.0; //分米转米
	}
	delta_y += sec_wall[cur_row][cur_column] / 10.0 / 2.0; //当前砖块的一般距离
	double y = 4.0 - delta_y + 0.1;//当前砖块的中心的x坐标，墙体长4米；+0.1是因为第二面墙顶着第一面墙，没有接触原点
	double z = cur_row * 0.2 + 0.2;//z等于这层及以下所有砖块的高度,砖块高度为0.2m
	vector<double> res{ x,y,z, double(current_brick) / 10.0 };//转换单位为m

	//指针移动到下一块砖
	sec_wall_column++;
	if (sec_wall_column >= sec_wall[cur_row].size())//当前行宽度方向到头了
	{
		sec_wall_column = 0;
		sec_wall_row++;
	}

	return res;
}

bool BrickPlan::fir_wall_isFinished()
{
	//z方向高度大于墙体高度，则这面墙处理完毕
	if (fir_wall_row >= fir_wall.size()) {
		return true;
	}
	else
	{
		return false;
	}
}

bool BrickPlan::sec_wall_isFinished()
{
	if (sec_wall_row>=sec_wall.size())
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
	while (!brickplan.fir_wall_isFinished())
	{
		vector<double> cor = brickplan.get_fir_wall_nextbrick_xyz();
		for (size_t i = 0; i < cor.size(); i++)
		{
			cout << cor[i] << "  ";
		}
		cout << "\n";

	}
	cout << "第二面墙\n";
	int row = 0;
	while (!brickplan.sec_wall_isFinished())
	{
		cout << "行" << row << ": ";
		row++;
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
