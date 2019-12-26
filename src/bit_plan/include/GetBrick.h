// GetBrick2.0.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//砖块用RGBO表示，为颜色首字母缩写，R：0.3m, G:0.6m, B:1.2m; O:1.8m;

#include <iostream>
#include <vector>
#include <string>
using namespace std;

class BrickPlan
{
public:
	BrickPlan();
	~BrickPlan();
	//获得下一层（也就是下一车）的所有砖块该有的位置
	vector<vector<double>> get_next_layer();
	//获取wall_1某层所有砖的位置
	//param:1.某趟，车上载砖的顺序;2.层数 layer 0-4
	vector<vector<double>> get_specified_layer_of_wall_1(vector<string> brick_in_car, int layer);
	//获取wall_2某层所有砖的位置
	vector<vector<double>> get_specified_layer_of_wall_2(int layer);

public:
	vector<vector<string>> wall_1;
	vector<string> orange_brick_in_car;
	vector<string> rgb_brick_in_car;
};