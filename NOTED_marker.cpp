//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#include "marker.h"
#include <opencv2/opencv.hpp>

using namespace std;

/* NOTE:
 * After reading this file, seems to me that we need to switch to another implementation of
 * GetMarker function using Aruco, which is the interface of MarkerDetector class.
 * One thing to notice is that we need to obey on MarkerInfo struct, I don't understand some
 * of it's fields.
 *
 * MarkerInfo is defined in IMarker.h
 * Id is calculated by GetCode function, which I guess is used to identify same marker between frames
 * Corners seems to be the corners in 2d image
 * Points seems to be a constant?
 */

MarkerDetector::MarkerDetector()
{
	nMinSize = 100;
	nMaxSize = 1000000000;
	nThreshold = 120;
	dApproxPolyCoef = 0.12;

	dMarkerFrame = 0.4;
	nMarkerCorners = 5;

	bDraw = true;

	GetMarkerPointsForWarp(vPts);
}

/* NOTE:
 * Actual work to detect marker
 */
bool MarkerDetector::GetMarker(cv::Mat &img, MarkerInfo &marker)
{
        /* NOTE:
         * Seems first convert the image to grayscale
         * Then find the contours in the image - edge detection?
         */
	vector<MarkerInfo> markers;
	cv::Mat img2, img3;
	cv::cvtColor(img, img2, CV_BGR2GRAY);
	cv::threshold(img2, img2, nThreshold, 255, CV_THRESH_BINARY);

	img2.copyTo(img3);

	vector<vector<cv::Point>> contours;	
	cv::findContours(img3, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

        /* NOTE:
         * We then iterate over the list of contours
         * Contours seems to be a list of list
         * It contains a list of 'shapes', and each of them contains a list of 'corners'
         */
	for (unsigned int i = 0; i < contours.size(); i++)
	{
                /* NOTE:
                 * Calcualte area of the contour and filter the ones that are too large or small
                 */
		vector<cv::Point> corners;
		double area = cv::contourArea(contours[i]);

		if (area < nMinSize || area > nMaxSize)
			continue;

                /* NOTE:
                 * Not sure what this line does
                 */
		cv::approxPolyDP(contours[i], corners, sqrt(area)*dApproxPolyCoef, true);

                /* NOTE:
                 * Convert all the coners to floats? why? What are the types before?
                 */
		vector<cv::Point2f> cornersFloat;
		for (unsigned int j = 0; j < corners.size(); j++)
		{
			cornersFloat.push_back(cv::Point2f((float)corners[j].x, (float)corners[j].y));
		}

                /* NOTE:
                 * Seems to be testing if the current contour is possible to be a marker
                 * Like testing how many corners are in the detection
                 * I don't understand why they DON'T want the corners to be convex
                 * nMarkerCorners is 5 ... I don't understand why it's not 4
                 */
		if (!cv::isContourConvex(corners) && corners.size() == nMarkerCorners && OrderCorners(cornersFloat))
		{	
			bool order = true;

                        /* NOTE:
                         * This codes seems to be some kind of hash result used as id to represent marker
                         * My guess is that this code is used to identify the same marker between frames
                         */
			int code = GetCode(img2, vPts, cornersFloat);

                        /* NOTE:
                         * Another attempt to calculate that 'code'
                         * Reverse the order corners and recalculate code on error
                         */
			if (code < 0)
			{
				reverse(cornersFloat.begin() + 1, cornersFloat.end());
				code = GetCode(img2, vPts, cornersFloat);

				if (code < 0)
					continue;

				order = false;
			}

			//I have commented this out as it crashed for some people, if you want additional accuracy in calibration try to uncomment it.
			//CornersSubPix(cornersFloat, contours[i], order);

			vector<Point2f> cornersFloat2(nMarkerCorners);
			vector<Point3f> points3D;			

			for (int i = 0; i < nMarkerCorners; i++)
			{
				cornersFloat2[i] = Point2f(cornersFloat[i].x, cornersFloat[i].y);
			}

                        /* NOTE:
                         * This function push a constant to points3D
                         * I have no idea what's the point of doing it
                         */
			GetMarkerPoints(points3D);

                        /* NOTE:
                         * Successful detected a marker candidate
                         * The constructor of MarkerInfo is defined in IMarker.h
                         * It has some comments for what the fields in this struct are, but is writtent in another language
                         */
			markers.push_back(MarkerInfo(code, cornersFloat2, points3D));

                        /* NOTE:
                         * Draw the border of the marker
                         */
			if (bDraw)
			{
				for (unsigned int j = 0; j < corners.size(); j++)
				{
					cv::circle(img, cornersFloat[j], 2, cv::Scalar(0, 50 * j, 0), 1);
					cv::line(img, cornersFloat[j], cornersFloat[(j + 1) % cornersFloat.size()], cv::Scalar(0, 0, 255), 2);
				}
			}
		}
	}

        /* NOTE:
         * Find the one with the maximum area out of marker candidates and draw it
         */
	if (markers.size() > 0)
	{
		double maxArea = 0;
		int maxInd = 0;

		for (unsigned int i = 0; i < markers.size(); i++)
		{
                        /* NOTE:
                         * They seem stupidly calculate the same thing twice
                         */
			if (GetMarkerArea(markers[i]) > maxArea)
			{
				maxInd = i;
				maxArea = GetMarkerArea(markers[i]);
			}
		}

		marker = markers[maxInd];
		if (bDraw)
		{
			for (int j = 0; j < nMarkerCorners; j++)
			{
				cv::Point2f pt1 = cv::Point2f(marker.corners[j].X, marker.corners[j].Y);
				cv::Point2f pt2 = cv::Point2f(marker.corners[(j + 1) % nMarkerCorners].X, marker.corners[(j + 1) % nMarkerCorners].Y);
				cv::line(img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
			}
		}

		return true;
	}
	else
		return false;
}


/* NOTE:
 * This function is the interface to get marker
 * It passes in an image (represented by pointer and size)
 * The results are saved to image and marker
 * Returns if the detection is successful
 *
 * This function then convert the raw image representation to CV representation
 * The actual work is done in another GetMarker function
 * Then convert the CV representation back into the raw representation
 *
 * Why don't use CV image directly?
 */
bool MarkerDetector::GetMarker(RGB *img, int height, int width, MarkerInfo &marker)
{ 
	cv::Mat cvImg(height, width, CV_8UC3);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			cvImg.at<cv::Vec3b>(i, j)[0] = img[j + width * i].rgbBlue;
			cvImg.at<cv::Vec3b>(i, j)[1] = img[j + width * i].rgbGreen;
			cvImg.at<cv::Vec3b>(i, j)[2] = img[j + width * i].rgbRed;
		}
	}

	bool res = GetMarker(cvImg, marker);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			img[j + width * i].rgbBlue = cvImg.at<cv::Vec3b>(i, j)[0];
			img[j + width * i].rgbGreen = cvImg.at<cv::Vec3b>(i, j)[1];
			img[j + width * i].rgbRed = cvImg.at<cv::Vec3b>(i, j)[2];
		}
	}

	return res;
}

/* NOTE:
 * This function seems to be ordering corners
 * But I didn't find the actual 'ordering' code
 */
bool MarkerDetector::OrderCorners(vector<cv::Point2f> &corners)
{
        /* NOTE:
         * Hull is the smallest poly that 'surrounds' a set of points
         * See the definition of convext hull: https://en.wikipedia.org/wiki/Convex_hull
         * It outputs a list of indices indicating their original index in corners
         */
	vector<int> hull;

	cv::convexHull(corners, hull);

	if (hull.size() != corners.size() - 1)
		return false;

        /* NOTE:
         * Check if a corner point is in the hull
         * Index becomes the first pointer that's not in the hull
         */
	int index = -1;
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		bool found = false;
		for (unsigned int j = 0; j < hull.size(); j++)
		{
			if (hull[j] == i)
			{
				found = true;
				break;
			}
		}

		if (!found)
		{
			index = i;
			break;
		}
	}

        /* NOTE:
         * Rotate the corners with pivot being the first one that's not in the hull?
         * I don't get why this is 'ordering' corners
         */
	vector<cv::Point2f> corners2;
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		corners2.push_back(corners[(index + i)%corners.size()]);
	}

	corners = corners2;
	return true;
}

/* NOTE:
 * This function is almost pure math, seems to be using some algorithm
 * Returns -1 on error
 */
int MarkerDetector::GetCode(cv::Mat &img, vector<cv::Point2f> points, vector<cv::Point2f> corners)
{
	cv::Mat H, img2;
	
	int minX = 0, minY = 0;

	double markerInterior = 2 - 2 * dMarkerFrame;

	for (unsigned int i = 0; i < points.size(); i++)
	{
		points[i].x = static_cast<float>((points[i].x - dMarkerFrame + 1) * 50);
		points[i].y = static_cast<float>((points[i].y - dMarkerFrame + 1) * 50);
	}

	H = cv::findHomography(corners, points);
	cv::warpPerspective(img, img2, H, cv::Size((int)(50 * markerInterior), (int)(50 * markerInterior)));

	int xdiff = img2.cols / 3;
	int ydiff = img2.rows / 3;
	int tot = xdiff * ydiff;
	int vals[9];

	cv::Mat integral;
	cv::integral(img2, integral);
	
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int temp;
			temp = integral.at<int>((i + 1) * xdiff, (j + 1) * ydiff);
			temp += integral.at<int>(i * xdiff, j * ydiff);
			temp -= integral.at<int>((i + 1) * xdiff, j * ydiff);
			temp -= integral.at<int>(i * xdiff, (j + 1) * ydiff);

			temp = temp / tot;

			if (temp < 128)
				vals[j + i * 3] = 0;
			else if (temp >= 128)
				vals[j + i * 3] = 1;
		}
	}

	int ones = 0;
	int code = 0;
	for (int i = 0; i < 4; i++)
	{
		if (vals[i] == vals[i + 4])
			return -1;
		else if (vals[i] == 1)
		{
			code += static_cast<int>(pow(2, (double)(3 - i)));
			ones++;
		}
	}
	
	if (ones / 2 == (float)ones / 2.0)
	{
		if (vals[8] == 0)
			return -1;
	}

	if (ones / 2 != ones / 2.0)
	{
		if (vals[8] == 1)
			return -1;
	}
	
	return code;
}

/* NOTE:
 * Seems to be another process of calibration to increase accuracy
 */
void MarkerDetector::CornersSubPix(vector<cv::Point2f> &corners, vector<cv::Point> contour, bool order)
{
	int *indices = new int[corners.size()];
	
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		for (unsigned int j = 0; j < contour.size(); j++)
		{
			if (corners[i].x == contour[j].x && corners[i].y == contour[j].y)
			{
				indices[i] = j;
				break;
			}
		}
	}

	vector<cv::Point> *pts = new vector<cv::Point>[corners.size()];

	for (unsigned int i = 0; i < corners.size(); i++)
	{
		int index1, index2;
		if (order)
		{
			index1 = indices[i];
			index2 = indices[(i + 1) % corners.size()];
		}
		else
		{
			index1 = indices[(i + 1) % corners.size()];
			index2 = indices[i];
		}

		if (index1 < index2)
		{
			pts[i].resize(index2 - index1);
			copy(contour.begin() + index1, contour.begin() + index2, pts[i].begin());
		}
		else
		{
			pts[i].resize(index2 + contour.size() - index1);
			copy(contour.begin() + index1, contour.end(), pts[i].begin());
			copy(contour.begin(), contour.begin() + index2, pts[i].end() - index2);
		}
	}

	cv::Vec4f *lines = new cv::Vec4f[corners.size()];
	
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		cv::fitLine(pts[i], lines[i], CV_DIST_L2, 0, 0.01, 0.01);
	}

	vector<cv::Point2f> corners2;
	for (unsigned int i = corners.size() - 1; i < 2 * corners.size() - 1; i++)
		corners2.push_back(GetIntersection(lines[(i + 1) % corners.size()], lines[i % corners.size()]));

	corners = corners2;
}

cv::Point2f MarkerDetector::GetIntersection(cv::Vec4f lin1, cv::Vec4f lin2)
{
	float c1 = lin2[2] - lin1[2];
	float c2 = lin2[3] - lin1[3];
	float a1 = lin1[0];
	float a2 = lin1[1];
	float b1 = -lin2[0];
	float b2 = -lin2[1];

	cv::Mat A(2, 2, CV_32F);
	cv::Mat b(2, 1, CV_32F);
	cv::Mat dst(2, 1, CV_32F);

	A.at<float>(0, 0) = a1;
	A.at<float>(0, 1) = b1;
	A.at<float>(1, 0) = a2;
	A.at<float>(1, 1) = b2;
	b.at<float>(0, 0) = c1;
	b.at<float>(1, 0) = c2;

	//rozwi¹zuje uk³ad równañ
	cv::solve(A, b, dst);

	cv::Point2f res(dst.at<float>(0, 0) * lin1[0] + lin1[2], dst.at<float>(0, 0) * lin1[1] + lin1[3]);
	return res;
}

void MarkerDetector::GetMarkerPoints(vector<Point3f> &pts)
{
	pts.push_back(Point3f(0.0f, -1.0f, 0.0f));
	pts.push_back(Point3f(-1.0f, -1.6667f, 0.0f));
	pts.push_back(Point3f(-1.0f, 1.0f, 0.0f));
	pts.push_back(Point3f(1.0f, 1.0f, 0.0f));
	pts.push_back(Point3f(1.0f, -1.6667f, 0.0f));
}

void MarkerDetector::GetMarkerPointsForWarp(vector<cv::Point2f> &pts)
{
	pts.push_back(cv::Point2f(0, 1));
	pts.push_back(cv::Point2f(-1, 1.6667f));
	pts.push_back(cv::Point2f(-1, -1));
	pts.push_back(cv::Point2f(1, -1));
	pts.push_back(cv::Point2f(1, 1.6667f));
}

double MarkerDetector::GetMarkerArea(MarkerInfo &marker)
{
	cv::Mat hull;

	vector<cv::Point2f> cvCorners(nMarkerCorners);
	for (int i = 0; i < nMarkerCorners; i++)
	{
		cvCorners[i] = cv::Point2f(marker.corners[i].X, marker.corners[i].Y);
	}

	cv::convexHull(cvCorners, hull);
	return cv::contourArea(hull);
}

