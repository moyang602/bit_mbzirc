#include "GetBrick.h"

# define Orange_tolerance 0.1f	// MAX 0.2
# define RGB_tolerance 0.05f		// MAX 0.057
# define OLength 1.8f
# define RLength 0.3f
# define GLength 0.6f
# define BLength 1.2f
# define Xoffset -0.2f			
# define Yoffset 0.2f			// 0.5*0.4
# define BrickHeight 0.2f

vector<vector<double>> BrickPlan::get_specified_layer_of_wall_1(vector<string> brick_in_car, int layer) 
{
	vector<vector<double>> res;
	//wall_1指定层的砖块排布，接下来的问题就是找到specified_layer_in_wall与brick_in_car的对应关系
	vector<string> specified_layer_in_wall = wall_1[layer];
	//考虑车上的每一块砖
	for (int i = 0; i < brick_in_car.size(); i++)
	{
		double x = 0, y = 0, z = BrickHeight*(layer+1);//layer从0-4，
		double deltaTemp = 0;//x方向的偏移量
		//在墙体上找对应的砖块
		for (int j = 0; j < specified_layer_in_wall.size(); j++)
		{
			//找到对应颜色的砖
			if (specified_layer_in_wall[j] == brick_in_car[i])
			{
				//根据砖块颜色计算x
				switch (specified_layer_in_wall[j][0])
				{
				case 'R':
					x = Xoffset + RGB_tolerance + 0.5*RLength + deltaTemp;
					break;
				case 'G':
					x = Xoffset + RGB_tolerance + 0.5*GLength + deltaTemp;
					break;
				case 'B':
					x = Xoffset + RGB_tolerance + 0.5*BLength + deltaTemp;
					break;
				default:
					break;
				}
				vector<double> current_brick{ x,y,z };
				res.push_back(current_brick);
				//墙体这个位置置N，以免影响下次判断，改变的是局部变量，不影响this->wall_1
				specified_layer_in_wall[j] = "N";//N is Null
				break;//跳出这个循环，开始找第二块砖
			}
			else 
			{
				//没找到对应颜色的，deltaTemp增加对应距离
				switch (wall_1[layer][j][0])
				{
				case 'R':
					deltaTemp += RLength + RGB_tolerance;
					break;
				case 'G':
					deltaTemp += GLength + RGB_tolerance;
					break;
				case 'B':
					deltaTemp += BLength + RGB_tolerance;
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
			for (size_t i = 0; i <3; i++)
			{
				vector<double> brick1{ 0.0, Yoffset+1*Orange_tolerance+0.5*OLength, (i+1)*BrickHeight };
				res.push_back(brick1);
			}
			for (size_t i = 0; i <3; i++)
			{
				vector<double> brick2{ 0.0, Yoffset+2*Orange_tolerance+1.5*OLength, (i+1)*BrickHeight };
				res.push_back(brick2);
			}
			break;

		case 1:
			for (size_t i = 3; i < 5; i++)
			{
				vector<double> brick1{ 0.0, Yoffset+1*Orange_tolerance+0.5*OLength, (i+1)*BrickHeight };
				res.push_back(brick1);
			}
			for (size_t i = 3; i < 5; i++)
			{
				vector<double> brick2{ 0.0, Yoffset+2*Orange_tolerance+1.5*OLength, (i+1)*BrickHeight };
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

	vector<string> second_tempt{ "R","R","R","R","B","G","G"};
	rgb_brick_in_car = second_tempt;
}

BrickPlan::~BrickPlan()
{
}

