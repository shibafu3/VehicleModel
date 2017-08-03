#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"C:\\opencv\\build\\x86\\vc12\\lib\\opencv_world300d.lib")            // opencv_core
#else
//Releaseモードの場合
#pragma comment(lib,"C:\\opencv\\build\\x86\\vc12\\lib\\opencv_world300.lib") 
#endif

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
/*数値計算ライブラリ*/
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
/*可変長リストライブラリ*/
#include <list>
#include <vector>
/*OpenCVライブラリ*/
#include "opencv/cv.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/opencv.hpp"
#include "opencv2/superres/optical_flow.hpp"

#include <windows.h>

using namespace cv;
using namespace std;

class Vehicle {
	Point2d rear = {};
	Point2d front = {};
	double theta = 0;
	double wheel_base = 0;
	double wheel_angle = 0;
	double velocity = 0;
public : 
	Vehicle(Point2d &rear_in, double theta_in = 0, double wheel_base_in = 20, double velocity_in = 1){
		rear = rear_in;
		front.x = rear_in.x + wheel_base_in * cos(theta_in);
		front.y = rear_in.y + wheel_base_in * sin(theta_in);
		theta = theta_in;
		wheel_base = wheel_base_in;
		velocity = velocity_in;
	}
	Vehicle(Point &rear_in, double theta_in = 0, double wheel_base_in = 20, double velocity_in = 1){
		rear = rear_in;
		front.x = rear_in.x + wheel_base_in * cos(theta_in);
		front.y = rear_in.y + wheel_base_in * sin(theta_in);
		theta = theta_in;
		wheel_base = wheel_base_in;
		velocity = velocity_in;
	}
	int SetWheelAngel(double wheel_angle_in){
		wheel_angle = wheel_angle_in;
		return 1;
	}
	int SetVelocity(double velocity_in){
		velocity = velocity_in;
		return 1;
	}
	int DriveVehicle(){
		front.x = rear.x + wheel_base * cos(theta);
		front.y = rear.y + wheel_base * sin(theta);

		rear.x = rear.x + velocity  * cos(theta);
		rear.y = rear.y + velocity  * sin(theta);
		front.x = front.x + velocity  * cos(theta + wheel_angle);
		front.y = front.y + velocity  * sin(theta + wheel_angle);

		theta = atan2(front.y - rear.y, front.x - rear.x);
		return 1;
	}
	Point GetRearPoint(){
		Point rear_out;
		rear_out.x = round(rear.x);
		rear_out.y = round(rear.y);
		return rear_out;
	}
	Point GetFrontPoint(){
		Point front_out;
		front_out.x = round(front.x);
		front_out.y = round(front.y);
		return front_out;
	}
	double GetTheta(){
		return theta;
	}

};
class Viewer{
public:
	Mat image;
	Viewer(Size image_size, int type, Scalar color){
		image = Mat(image_size, type, color);
	}
	void SetPoint(Point point, Scalar color){
		for (int i = 0; i < 3; i++){
			image.at<Vec3b>(point)[i] = color[i];
			image.at<Vec3b>(Point(round(point.x + 1), round(point.y    )))[i] = color[i];
			image.at<Vec3b>(Point(round(point.x)    , round(point.y + 1)))[i] = color[i];
			image.at<Vec3b>(Point(round(point.x + 1), round(point.y + 1)))[i] = color[i];
			image.at<Vec3b>(Point(round(point.x - 1), round(point.y	   )))[i] = color[i];
			image.at<Vec3b>(Point(round(point.x)    , round(point.y - 1)))[i] = color[i];
			image.at<Vec3b>(Point(round(point.x - 1), round(point.y - 1)))[i] = color[i];
			image.at<Vec3b>(Point(round(point.x + 1), round(point.y - 1)))[i] = color[i];
			image.at<Vec3b>(Point(round(point.x - 1), round(point.y + 1)))[i] = color[i];
		}
	}
	void SetLine(Point line_start, Point line_end, Scalar color){
		line(image, line_start, line_end, color);
	}
	void Initial(int color){
		image = Scalar(color);
	}
	void Show(){
		imshow("image", image);
	}
};
class Target{
	vector<Point2d> route;
	vector<double> tangent;
	vector<int> flag;
	int target_num;
	int route_num;
public :
	void CreateCircleRoute(Point center, int radius, double clothoid = 0){
		route_num = -1;
		for (double i = M_PI; i > 0; i -= 0.1){
			route.push_back(Point(center.x + radius * cos(i), center.y + radius * sin(i)));
			radius = radius + clothoid;
			route_num++;
		}
		tangent = vector<double>(route_num);
		flag = vector<int>(route_num);
		for (int i = 0; i < route_num - 1; i++){
			tangent[0] = atan2(route[i].y - route[i + 1].y, route[i].x - route[i + 1].x);
		}
	}

	void CalcTarget(Vehicle &car){
		for (int i = 0; i < route_num; i++){
			if (route[i].y >= (-1 / tan(car.GetTheta()))*(route[i].x - car.GetRearPoint().x) + car.GetRearPoint().y){
				flag[i] = 1;
			}
			else {
				flag[i] = 0;
			}
		}

		for (int i = 0; i < route_num; i++){
			if (flag[i] != flag[i + 1]){
				target_num = i + 1;
				break;
			}
		}
	}
	double CalcParam(Vehicle &car){
		double d = 0;
		double nasukaku = 0;
		d = (-tan(tangent[target_num])*car.GetRearPoint().x + car.GetRearPoint().y + tan(tangent[target_num])*route[target_num].x - route[target_num].y) / (sqrt(tan(tangent[target_num])*tan(tangent[target_num]) + 1));
		nasukaku = car.GetTheta() - tangent[target_num];
		return nasukaku * 100 + d * 10;
	}

};
class GeneratePath{
	int point_number = 10;

	Point2d ego_point;
	double ego_theta;
	Point2d ego_circle_center;
	Point2d ego_e;

	Point2d target_point;
	double target_theta;
	Point2d target_circle_center;

	Point2d ego_to_target;
	double first_turning_radius;
	double min_turning_radius;

	vector<Point2d> path;
	vector<Point> pathi;
	vector<Point2d> path_vector;
	vector<Point2d> path_vector_vector;

	double first_radian_start;
	double first_radian_end;
	double first_radian_step;

	double second_radian_start;
	double second_radian_end;
	double second_radian_step;

	double SubtractRadian(double radian1, double radian2){
		double radian_out = radian1 - radian2;
		while (radian_out < -M_PI){
			radian_out = radian_out + 2 * M_PI;
		}
		while (radian_out > M_PI){
			radian_out = radian_out - 2 * M_PI;
		}
		return radian_out;
	}
	double OffSetRadian(double radian1, double radian2){
		double radian_out = radian1 - radian2;
		while (radian_out < 0){
			radian_out = radian_out + 2 * M_PI;
		}
		while (radian_out >= 2 * M_PI){
			radian_out = radian_out - 2 * M_PI;
		}
		return radian_out;
	}
	double MyAtan2(double radian1, double radian2){
		if (atan2(radian1, radian2) < 0){
			return atan2(radian1, radian2) + (2 * M_PI);
		}
		else{
			return atan2(radian1, radian2);
		}
	}

public :
	void SetEgo(Point2d point, double theta){
		ego_point = point;
		ego_theta = theta;
	}
	void SetTarget(Point2d point, double theta){
		target_point = point;
		target_theta = theta;
	}	
	void SetMinRadius(double radius){
		min_turning_radius = radius;
	}
	vector<Point2d> &GetPath(){
		target_circle_center = Point2d(target_point.x + min_turning_radius * cos(target_theta - M_PI / 2), 
									   target_point.y + min_turning_radius * sin(target_theta - M_PI / 2));
		ego_e = Point2d(cos(ego_theta + M_PI / 2),
						sin(ego_theta + M_PI / 2));
		ego_to_target = ego_point - target_circle_center;
		// (l^2 - |c|^2) / 2(e*c - l)
		first_turning_radius = (min_turning_radius * min_turning_radius - ego_to_target.ddot(ego_to_target)) / (2 * (ego_e.dot(ego_to_target) - min_turning_radius));
		ego_circle_center = ego_point + ego_e * first_turning_radius;
		path = vector<Point2d>(2 * point_number + 1);

		first_radian_start = OffSetRadian(ego_theta, -M_PI / 2);
		first_radian_end = MyAtan2(target_circle_center.y - ego_circle_center.y, target_circle_center.x - ego_circle_center.x);
		first_radian_step = SubtractRadian(first_radian_end, first_radian_start) / point_number;

		second_radian_start = MyAtan2(ego_circle_center.y - target_circle_center.y, ego_circle_center.x - target_circle_center.x);
		second_radian_end = OffSetRadian(target_theta, -M_PI * 3 / 2);
		second_radian_step = SubtractRadian(second_radian_end, second_radian_start) / point_number;

		double t = first_radian_start;
		for (int i = 0; i < point_number; i++){
			path[i] = Point(ego_circle_center.x + first_turning_radius * cos(t),
							ego_circle_center.y + first_turning_radius * sin(t));
			t += first_radian_step;
		}
		t = second_radian_start;
		for (int i = point_number; i < 2 * point_number + 1; i++){
			path[i] = Point(target_circle_center.x + min_turning_radius * cos(t),
							target_circle_center.y + min_turning_radius * sin(t));
			t += second_radian_step;
		}

		path_vector = vector<Point2d>(path.size() - 1);
		for (int i = 0; i < path_vector.size(); i++){
			path_vector[i] = Point(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
		}

		path_vector_vector = vector<Point2d>(path_vector.size() - 1);
		for (int i = 0; i < path_vector_vector.size(); i++){
			path_vector_vector[i] = Point(path_vector[i + 1].x - path_vector[i].x, path_vector[i + 1].y - path_vector[i].y);
		}
		return path;
	}
	vector<Point> &GetPathi(){
		pathi = vector<Point>(2 * point_number + 1);
		for (int i = 0; i < 2 * point_number + 1; i++){
			pathi[i] = Point(round(path[i].x), round(path[i].y));
		}
		return pathi;
	}
	vector<Point2d> &GetPathVector(){
		return path_vector;
	}
	vector<Point2d> &GetPathVectorVector(){
		return path_vector_vector;
	}
};
class ClothoidCurve{
	Point point_start;
	Point vector_start;
	double radius_start;
	double curvature_start;

	Point point_goal;
	Point vector_goal;
	double radius_goal;
	double curvature_goal;

	vector<Point2d> path;

public :
	void SetPoint(Point start, Point goal){
		point_start = start;
		point_goal = goal;
	}
	void SetAngle(double start, double goal){
		radius_start = start;
		radius_goal = goal;
	}
	void SetCurvature(double start, double goal){
		curvature_start = start;
//		curvature_goal = (radius_goal  * radius_goal) / 2;
	}
	void SetVector(Point start, Point goal){
		vector_start = start;
		vector_goal = goal;
	}
	void CulcVector(Point start, Point goal){
		vector_start = Point(cos(radius_start), sin(radius_start));
		vector_goal = Point(cos(radius_goal), sin(radius_goal));
	}
	void CreatePath(){
		int S = 0;
		for (double i = radius_start; i < radius_goal; i += 0.1){
			
		}
	}

};



/*
template <typename vec> class MathMatrix{
	vec X, Y;

public :
	MathMatrix(vec X_in, vec Y_in){
		X = X_in;
		Y = Y_in;
	}
	double operator*(MathMatrix &obj){
		return X * obj.X + Y * obj.Y;
	}
	MathMatrix &operator*(vec i){
		X = X * i;
		Y = Y * i;
		return *this;
	}
	MathMatrix &operator+(MathMatrix &obj){
		X = X + obj.X;
		Y = Y + obj.Y;
		return *this;
	}
	MathMatrix operator+(vec i){
		X = X + i;
		Y = Y + i;
		return *this;
	}


};
*/

int main(){
	Viewer image(Size(1024, 1024), CV_8UC3, Scalar::all(0));
	Vehicle car(Point(412, 512), -90 * M_PI / 180, 20, -1);

	GeneratePath parallel;

	parallel.SetEgo(Point(0 + 512, 0 + 512), 0);
	parallel.SetTarget(Point(80 + 512, 40 + 512), 0);
	parallel.SetMinRadius(60);

	while (1){
		image.Initial(0);

		image.SetLine(Point(0, 511), Point(1023, 511), Scalar::all(255));
		image.SetLine(Point(511, 0), Point(511, 1023), Scalar::all(255));


		Target route;
		route.CreateCircleRoute(Point(512, 512), 100);


		double root[100][4] = {};
		int j = 0, kuro = 100;
		for (double i = M_PI; i > 0; i -= 0.1){
			root[j][0] = 512 + kuro * cos(i);
			root[j][1] = 512 + kuro * sin(i);
			image.SetPoint(Point(round(root[j][0]), round(root[j][1])), Scalar(255, 0, 0));
			j++;
//			kuro -= 1;
		}
		for (int i = 0; i < j; i++){
			root[i][2] = atan2(root[i][1] - root[i + 1][1], root[i][0] - root[i + 1][0]);
		}

		car.DriveVehicle();

		image.SetPoint(car.GetFrontPoint(), Scalar::all(255));
		image.SetPoint(car.GetRearPoint(), Scalar(0, 0, 255));


		for (int i = 0; i < j + 1; i++){
			if (root[i][1] >= (-1 / tan(car.GetTheta()))*(root[i][0] - car.GetRearPoint().x) + car.GetRearPoint().y){
				root[i][3] = 1;
			}
			else {
				root[i][3] = 0;
			}
		}

		for (int i = 0; i < j; i++){
			if (root[i][3] != root[i + 1][3]){
				j = i + 1;
				break;
			}
		}

		
		double d = 0;
		double nasukaku = 0;
		double delta = 0;
		d = (-tan(root[j][2])*car.GetRearPoint().x + car.GetRearPoint().y + tan(root[j][2])*root[j][0] - root[j][1]) / (sqrt(tan(root[j][2])*tan(root[j][2]) + 1));


		nasukaku = car.GetTheta() - root[j][2];


		if (nasukaku < 0){ nasukaku += 2 * M_PI; }

		delta = nasukaku * 100 / (abs(d) * 5 + 1) + d * 5;


		delta = nasukaku * 100 + d * 10;
		delta = d *10;

		delta = min(max(delta, -30), 30);
		delta = delta * M_PI / 180;
		car.SetWheelAngel(delta);




		cout << j << " 距離:" << d << " なす角:" << nasukaku * 180 / M_PI << " 出力:" << delta * 180 / M_PI << "\n";


		Sleep(50);
		image.Show();
		if (waitKey(10) >= 0) { break; }
	}

	vector<Point2d> path = parallel.GetPath();
	image.Initial(0);

	for (int i = 0; i < 21; i++){
		std::cout << path[i] << endl;
		image.SetPoint(path[i], Scalar(255, 255, 255));
	}

	image.Show();
	waitKey(0);


	getchar();


	return 0;
}