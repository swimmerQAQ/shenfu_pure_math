#include <opencv2/opencv.hpp>
 using namespace cv;
 #include <iostream>
 using namespace std;
 #include <math.h>
 
 Mat imgHSV;//frame transfer imghsv
 Mat mask;//for inrang after imghsv
 Mat imgGray, imgBlur, imgCanny, imgDil, imgErode;
 Mat medium;

 int hmin = 90;
 int hmax = 135;
 int smin = 106;
 int smax = 255;
 int vmin = 43;
 int vmax = 255;
 //vector<Point> kval(5);// store the K value
  float distance(Point A,Point B)
 {
 	float bulf[2];
 	bulf[0] = powf((A.x - B.x),2);
 	bulf[1] = powf((A.y - B.y),2);
 	return sqrtf(bulf[0]+bulf[1]);
 }
 Point midPoint(Point A, Point B)
 {
 	Point bulfPoint;
 	bulfPoint.x = (A.x + B.x)/2 ;
 	bulfPoint.y = (A.y + B.y)/2 ;
 	return bulfPoint;
 }
 void detect(Mat in_img,Mat origin)
 {
 	vector<vector<Point>> contours ;
	vector<Vec4i> hierarchy;//后前子父 -1 nonesense
	findContours( in_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	vector<vector<Point>> fourAngle(contours.size());
	vector<Point> centre(contours.size());
	//vector<vector<Point>> conPoly(contours.size());//画点
	int radius[contours.size()]={0};
	cout << "begin_detect"<<endl;
	vector<int> compare(contours.size());
	vector<int> area(contours.size());
	for(int i=0;i<contours.size();i++)
	{
		
		//四角点储存，与conpoly重复，但后面用了很多，没必要改
		/*
		想了想，还是改一下，艹
		放入每个检索到的，图形的点
		由于必须是每个(第i个)封闭面内，要在循环内
		*/
		fourAngle[i] = contours[i];
		
		/*
		上面这一步非常重要，不初始化使用数组存在超范围数组调用，导致程序崩溃
		*/
		drawContours(origin, contours, i,Scalar(75,100,255),1);
		for(int j=0;j<contours[i].size();j++)//check all points
		{
			/*
				find_up_point
				the min yyyyyy
			*/
			area[i] = contourArea(contours[i]);
			if(fourAngle[i][0].y >= contours[i][j].y)
			{
				fourAngle[i][0].y = contours[i][j].y;
				fourAngle[i][0].x = contours[i][j].x;
			}
			/*
				find_down_point
				the max yyyyyy
			*/
			if(fourAngle[i][1].y < contours[i][j].y)
			{
				fourAngle[i][1].y = contours[i][j].y;
				fourAngle[i][1].x = contours[i][j].x;
			}
			/*
				find_left_point
				the min xxxxx
			*/
			if(fourAngle[i][2].x >= contours[i][j].x)
			{
				fourAngle[i][2].x = contours[i][j].x;
				fourAngle[i][2].y = contours[i][j].y;
			}
			/*
				find_right_point
				the max xxxxx
			*/
			if(fourAngle[i][3].x < contours[i][j].x)
			{
				fourAngle[i][3].x = contours[i][j].x;
				fourAngle[i][3].y = contours[i][j].y;
			}
			/*
			
			
			
			
			
			
				fourAngle[i][0]
					#
			 	##############
		  fourAngle[i][2]#################fourAngle[i][3]
			 	##############
					#
				foourAngle[i][1]
				
				
				
				
				
				
				
			*/
		}
		
		
		
		//rectangle(img, conPoly[i][0], conPoly[i][1], Scalar(100, 255, 50),1);
		//line(origin, fourAngle[i][0], fourAngle[i][1], Scalar(0, 255, 0),2);
		//中点
		//非常重要，如果没有多维的radius和centre，始终只能测一个点，画一个圈，切记
		radius[i] = (fourAngle[i][1].y -fourAngle[i][0].y)/2;
		centre[i].y = (fourAngle[i][0].y+fourAngle[i][1].y)/2;
		centre[i].x = (fourAngle[i][2].x+fourAngle[i][3].x)/2;
		//rectangle(origin, Point(fourAngle[i][2].x,fourAngle[i][1].y), Point(fourAngle[i][3].x, fourAngle[i][0].y), Scalar(0, 200, 100),2);
		cout << "dist = " << distance(Point(fourAngle[i][2].x,fourAngle[i][1].y), Point(fourAngle[i][3].x, fourAngle[i][0].y)) << endl;
	}
	cout << " num_of obj "<< contours.size() <<endl;
	for(int i=0;i<contours.size();i++)
	{
		//遍历所有的点
		compare[i] = fabs(fourAngle[i][1].y-fourAngle[i][0].y);
		compare[i] = fabs(fourAngle[i][3].x-fourAngle[i][2].x);
		cout << " detect whether is circle " << fabs(compare[i]-compare[i])<< endl;
		cout << " the area is " << area[i] << endl;
		
		if(fabs(compare[i]-compare[i])<200)
		{
			//两边直径只差尽量小
			if(area[i]<450 && area[i]>300)
			{
				//面积锁定圆心
				circle(origin,centre[i],radius[i], Scalar(0, 0, 255),FILLED);
				rectangle(origin, Point(fourAngle[i][2].x,fourAngle[i][1].y), Point(fourAngle[i][3].x, fourAngle[i][0].y), Scalar(0, 200, 100),2);
				//开始遍历其他图形，寻找合适的
				for(int j=0;j<contours.size();j++)
		 		{
		 			if(j != i)
		 			{
		 		 		if(area[j]>3400 && area[j]<5000)//check the big j_th contours
						{
							//找到合适的图形
							cout << "the big area = " << area[j] << endl;
							rectangle(origin, Point(fourAngle[j][2].x,fourAngle[j][1].y), Point(fourAngle[j][3].x, fourAngle[j][0].y), Scalar(0, 200, 100),2);
							circle(origin,centre[i],4, Scalar(255, 255, 0),FILLED);
							circle(origin,centre[j],4, Scalar(255, 255, 0),FILLED);
							
							//寻找图形内部的点，希望找到两个距离最大点
							float max_dist=0;//储存最大距离
							for(int k=0;k<contours[i].size();k++)
							{
								//distance(centre[i],contours[j][k])筛选距离——>圆心：第i个图形中心的距离；第j个（j！=i）的第k个点观察距离；
								//samesense because the type ——————> vector<Point>  vector<vector<Point>>
								if(max_dist < distance(centre[i],contours[j][k]))
								{
									max_dist = distance(centre[i],contours[j][k]);
								}
							}//找到最远距离
							cout << "find max_dist = " << max_dist << endl;
							//再次进循环找点
							for(int k=0;k<contours[i].size();k++)
							{
								
									//寻找一定比例的点
									if(distance(centre[i],contours[j][k]) >= 125)
									{
										circle(origin,contours[j][k],4, Scalar(255, 0, 255),FILLED);
										//找到后发送距离到底是多少
										cout <<" ______________max_dist = "<< max_dist << endl;
										//line(origin, centre[j], contours[j][k], Scalar(0, 255, 255),3);
										circle(origin,midPoint(contours[j][k], centre[j]),7, Scalar(150, 255, 0),FILLED);
										
									}
							}
							
						}
		 			}
		 		}
			}
		}
	}

	//////////////////
	
 }
 int main()
 {
 	VideoCapture capture;
 	Mat frame;
 	string path = "a1.avi" ;
 	frame = capture.open(path);
 	if(!capture.isOpened())
 	{
 		cout << " cannot open the vedio..." << endl;
 		return -1;
 	}
 	namedWindow("Trackbars",1);
 	createTrackbar("Hue min ","Trackbars",&hmin,255);
	createTrackbar("Hue max ","Trackbars",&hmax,255);
	createTrackbar("sue min ","Trackbars",&smin,255);
	createTrackbar("sue max ","Trackbars",&smax,255);
	createTrackbar("vue min ","Trackbars",&vmin,255);
	createTrackbar("vue max ","Trackbars",&vmax,255);
 	while(capture.read(frame))
 	{
 		cvtColor(frame, imgHSV, COLOR_BGR2HSV);
 		Scalar lower(hmin, smin, vmin);
	 	Scalar upper(hmax, smax, vmax);
	 	inRange(imgHSV, lower, upper, mask);
	 	
 		imshow("HSV_img", mask);
 		cout << "hmin="<<hmin<<endl;
 		cout << "hmax="<<hmax<<endl;
 		cout << "smin="<<smin<<endl;
 		cout << "smax="<<smax<<endl;
 		cout << "vmin="<<vmin<<endl;
 		cout << "vmax="<<vmax<<endl;

 		GaussianBlur(mask, imgBlur, Size(3,3),3,0);
 		Canny(imgBlur, imgCanny, 25, 75);
 		imshow("imgcanny",imgCanny);
 		Mat kernal = getStructuringElement(MORPH_RECT, Size(5,5));
 		dilate(imgBlur, imgDil, kernal);
 		imshow("imgDil",imgDil);
 		erode(imgDil, imgErode, kernal);
 		
 		imshow("img_erode",imgErode);
 		//blur(imgErode, medium, Size(5,5), {-1,-1});
 		//imshow("medium", medium);
 		//erode(medium, medium, kernal);
 		detect(imgDil, frame);
 		imshow("detect_final",frame);
 		if(waitKey(30)==32)
 		{
 			if(waitKey()==32)
 			{
 				break;
 			}
 		}
 	}
	capture.release();
 	return 0;
 }
 
 
 
 
 
 
