#include "GetBrick.h"

vector<vector<double>> BrickPlan::get_specified_layer_of_wall_1(vector<string> brick_in_car, int layer) 
{
	vector<vector<double>> res;
	//wall_1指定层的砖块排布，接下来的问题就是找到specified_layer_in_wall与brick_in_car的对应关系
	vector<string> specified_layer_in_wall = wall_1[layer];
	//考虑车上的每一块砖
	for (int i=0;i<brick_in_car.size();i++)
	{
		double x = 0, y = 0, z = 0.2*layer+0.2;//layer从0-4，
		double x_offset = 0;//x方向的偏移量
		//在墙体上找对应的砖块
		for (int j=0;j<specified_layer_in_wall.size();j++)
		{
			//找到对应颜色的砖
			if (specified_layer_in_wall[j]==brick_in_car[i])
			{
				//墙体这个位置置N，以免影响下次判断，改变的是局部变量，不影响this->wall_1
				specified_layer_in_wall[j] = "N";//N is Null
				//根据砖块颜色计算x
				switch (specified_layer_in_wall[j][0])
				{
				case 'R':
					x = 4.0 - x_offset - 0.3 / 2;
					break;
				case 'G':
					x = 4.0 - x_offset - 0.6 / 2;
					break;
				case 'B':
					x = 4.0 - x_offset - 1.2 / 2;
					break;
				default:
					break;
				}
				vector<double> current_brick{ x,y,z };
				res.push_back(current_brick);
			}
			else {
				//没找到对应颜色的，x_offset增加对应距离
				switch (specified_layer_in_wall[j][0])
				{
				case 'R':
					x_offset += 0.3;
					break;
				case 'G':
					x_offset += 0.6;
					break;
				case 'B':
					x_offset += 1.2;
					break;
				default:
					break;
				}
			}
		}
	}
	return res;
}
vector<vector<double>> BrickPlan::get_specified_layer_of_wall_2(int layer) {
	vector<vector<double>> res;
	switch(layer)
	{
		case 0:
			for (size_t i = 0; i < 3; i++)
			{
				vector<double> brick1{ 0.1, 3.2, 0.2+i*0.2 };
				vector<double> brick2{ 0.1, 1.4, 0.2+layer*0.2 };
				res.push_back(brick1);
				res.push_back(brick2);
			}
			break;

		case 1:
			for (size_t i = 3; i < 5; i++)
			{
				vector<double> brick1{ 0.1, 3.2, 0.2+i*0.2 };
				vector<double> brick2{ 0.1, 1.4, 0.2+layer*0.2 };
				res.push_back(brick1);
				res.push_back(brick2);
			}
			break;
		default:
		break;
	}

	return res;
}

BrickPlan::BrickPlan()
{
	//行数代表层数，列数=0表示离角点最远的那块砖
	vector<vector<string>> first_tempt{
		vector<string>{"B","R","G","R","G","R","R"},
		vector<string>{"R","G","R","G","R","R","B"},
		vector<string>{"R","R","G","R","G","R","B"},
		vector<string>{"B","R","R","R","G","R","G"},
		vector<string>{"R","R","G","R","R","B","G"}
	};
	wall_1 = first_tempt;//只初始化wall_1，wall_2是固定的

	vector<string> rgb_bricks{ "B","R","G","R","G","R","R" };
}

BrickPlan::~BrickPlan()
{
}

