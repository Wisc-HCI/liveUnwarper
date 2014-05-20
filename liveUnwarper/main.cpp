//
//  main.cpp
//  liveUnwarper
//
//  Created by steven on 4/12/14.
//  Copyright (c) 2014 what. All rights reserved.
//

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <thread>
#include <opencv2/nonfree/nonfree.hpp>
#include <list>
#include <unistd.h>
using namespace std;
using namespace cv;

//###########################################################
int panoramaDegrees = 40; // 360, 180, or 40 !
//###########################################################

// framerate stuff
int currentFrameRate = 150;//125;
int currentTrackingFrameRate = 300;//205;
list<int> rateList(200, 1);
list<int> trackingRateList(200,1);
int pastFailureCounter = 200;
int pastTrackingFailureCounter = 200;
int actualCounter = 0;

// things to change based on camera
//int radiusInner = 65, radiusOuter = 190; // 960x540 instantWebcam
//int radiusInner = 90, radiusOuter = 400; // 1920x1080 ipWebcam
int radiusInner = 50, radiusOuter = 190; // 1024x768 ipWebcam

string templateFileName = "template960.jpg";
//string templateFileName = "template1024.jpg";

//int unwarpedW = 1300, unwarpedH = 240;
int unwarpedW = 1440, unwarpedH = 240;
//int unwarpedW = 1140, unwarpedH = 200;
//int unwarpedW = 770, unwarpedH = 180;


int leftBlack = 0;
int rightBlack = unwarpedW;


// debug stuffs
bool debug = false;
bool tmpdebug = false;
VideoWriter processedOut;
VideoWriter gradientOut;

Mat thePanorama;

string rightTurn = "osascript ../scripts/rightMove.scpt";
string leftTurn = "osascript ../scripts/leftMove.scpt";
string currentCommand = leftTurn;
string frameRateCommand = "python ../scripts/frameRateChanger.py ";

const double PI = 3.1415926535897932384626;

string windowname = "Double";
int failCount = 0;

//CvCapture* camera;
int degreesOfNormalCamera = 40;
int leftViewNormalCamera, rightViewNormalCamera;
Mat frame;
Mat templ;
Mat inputGrad;
int frameCount, maxFrameCount;

// unwarping parameters
int cx = -1, cy = -1;
double scaleX = 1.0, scaleY = 1.0;

float angularOffset = 0.0f; // angle (in radians) to start unwarping from

int scanSideDist = 60; // was 40
int remapCenterX;
int remapCenterY;

//string pathToVideoImages = "/Users/steven/Documents/madison/panoramic/steveB/tmp/";
//string pathToVideoImages = "/Users/steven/dev/liveUnwarper/scripts/tmp/";
string pathToVideoImages = "../scripts/tmp/";

string filepreamble = pathToVideoImages + "unwarp/image";
string miniFilePreamble = pathToVideoImages + "debug/";

// tracking variables
int trackingSliceWidth = 60;
Mat trackedImage = Mat(Size(trackingSliceWidth*2, unwarpedH), CV_8UC3);
bool tracking = false;
int degreesFromCenter;
int trackingThresh = (unwarpedW*degreesOfNormalCamera)/720.0;
int trackingTimesUnderThresh = 0;

SurfFeatureDetector detector(400);
Mat descriptors_pano, descriptors_track;
std::vector<KeyPoint> keypoints_pano, keypoints_track;
SurfDescriptorExtractor extractor;
FlannBasedMatcher matcher;
std::vector< DMatch > matches;
float xAverage = 0.0;
int commandsGiven;
int heuristicTurns;

int leftX, rightX;

bool trackingTraining = false;
int trackingTrainingIterations = 0;

void setUpTrackingTraining()
{
    trackingTraining = true;
    tracking = true;
    leftX = 0;
    rightX = unwarpedW/2 + 2*trackingSliceWidth;
    trackedImage = templ = imread("sampleTrackedImage.jpg");
    detector.detect( trackedImage, keypoints_track );
    extractor.compute( trackedImage, keypoints_track, descriptors_track );
}

string toString ( int Number )
{
    ostringstream ss;
    ss << Number;
    return ss.str();
}

void setFrameRate(int rate)
{
    // sets delay for sampling of image stream
    system((frameRateCommand + toString(rate) + " &").c_str());
    //cout << "frame rate set to: " << rate << endl;
}

string format(int i)
{
    if(i < 10){
        return "000000000" + toString(i);
    }
    if(i < 100){
        return "00000000" + toString(i);
    }
    if(i < 1000){
        return "0000000" + toString(i);
    }
    if(i < 10000){
        return "000000" + toString(i);
    }
    if(i < 100000){
        return "00000" + toString(i);
    }
    if(i < 1000000){
        return "0000" + toString(i);
    }
    if(i < 10000000){
        return "000" + toString(i);
    }
    if(i < 100000000){
        return "00" + toString(i);
    }
    if(i < 1000000000){
        return "0" + toString(i);
    }
    return toString(i);
}

Mat unwarpSimple(Mat src, int *cx, int *cy)
{
    Mat map_x, map_y, dst;
    
    // center point
    float warpedCx = *cx;
    float warpedCy = *cy;
    
    // create map_x and map_y the same size as src
    map_x.create( unwarpedH, unwarpedW, CV_32FC1 );
    map_y.create( unwarpedH, unwarpedW, CV_32FC1 );
    
    float circFactor = 0.0f + 2 * PI / (float) unwarpedW;
    
    for ( int dsty = 0; dsty < unwarpedH; ++dsty )
    {
        float y = ( (float) dsty / (float) unwarpedH );
        /*
         -6.8126940324309999e-003
         1.4801742851070974e+000
         -1.3996466911266976e+000
         9.3309779408445981e-001
         */
        
        //float yfrac = -.00681269403243 + 1.4801742851070974*y - 1.3996466911266976*y*y + .933097794084459*y*y*y;
        
        //float a = .185, b = .8184, c = -.0028;
        float a = .0, b = 1.0, c = 0.0;
        float yfrac = a*y*y + b*y + c;
        //float yfrac = -.5 + sqrt(2*y + .25);
        //float yfrac = MIN( 1.0f, MAX( 0.0f, y ) );
        
        float radius = yfrac * (radiusOuter-radiusInner) + radiusInner;
        
        for (int dstx = 0; dstx < unwarpedW; ++dstx )
        {
            float angle = ( (float) dstx * circFactor ) + angularOffset;
            
            // map the source pixel to the destination pixel
            map_x.at< float >( dsty, dstx ) = warpedCx + radius * cosf( angle );
            map_y.at< float >( dsty, dstx ) = warpedCy + radius * sinf( angle );
        }
    }
    
    dst.create( unwarpedH, unwarpedW, src.type() );
    
    remap( src, dst, map_x, map_y, CV_INTER_LINEAR );
    
    //imwrite("converted.jpg",dst);
    
    return dst;
}
bool first = true;

void estimateCenterSteve(int *cx, int*cy, Mat img_display)
{
    if ( img_display.rows == 0 || img_display.cols == 0)
    {
        cout << "invald image. cannot estimate center! " << img_display.rows << "x" << img_display.cols << endl;
        return;
    }
    int oldcx = *cx;
    int oldcy = *cy;
    
    Rect area;
    
    if((oldcx < 0 && oldcy < 0) || actualCounter%105==0)
    {
        // scan the whole frame
        remapCenterX = remapCenterY = 0;
        area = Rect(0, 0, img_display.cols, img_display.rows);
    }
    else
    {
        // scan based on previous values
        remapCenterX = min(img_display.cols - (templ.cols + 2*scanSideDist) - 1, max(0, oldcx - scanSideDist - (templ.cols/2)));
        remapCenterY = min(img_display.rows - (templ.rows + 2*scanSideDist) - 1, max(0, oldcy - scanSideDist - (templ.rows/2)));
        
        area = Rect(remapCenterX, remapCenterY, templ.cols + 2*scanSideDist, templ.rows + 2*scanSideDist);
    }
    
    Mat scanImage;
    try
    {
        scanImage = img_display(area).clone();
        //cout << "initial type: " << scanImage.type() << endl;
        cvtColor(scanImage, scanImage, CV_BGR2GRAY);
        //cout << "converted type: " << scanImage.type() << endl;
        cvtColor(scanImage, scanImage, COLOR_GRAY2BGR);
        //cout << "reconverted type: " << scanImage.type() << endl;
    }
    catch(Exception e)
    {
        cout << "error image's specs: " << img_display.rows << "x" << img_display.cols << endl;
    }
    
    Mat result;
    int match_method = CV_TM_SQDIFF_NORMED;
    
    int result_cols =  scanImage.cols - templ.cols + 1;
    int result_rows = scanImage.rows - templ.rows + 1;
    
    result.create(result_cols, result_rows, CV_32FC1);
    
    //Laplacian(img_display, inputGrad, CV_8UC1, 3, 1, 0, BORDER_DEFAULT);
    threshold( scanImage, inputGrad, 90, 255, 0 );
    //inputGrad.convertTo(inputGrad, CV_8U);
    
    //cout << "type: " << inputGrad.type() << "  " << templ.type() << endl;
    matchTemplate(inputGrad, templ, result, match_method);
    //normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() ); todo::: may want this back on if things behave strangely
    
    // localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    
    // for SQDIFF and SQDIFF_NORMED, the best matches are lower values. for all the other methods, the higher the better
    if(match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
    { matchLoc = minLoc; }
    else
    { matchLoc = maxLoc; }
    
    // revise matchLoc back to large image's coordinates. matchLoc is upper left corner of match in the scanImage coords
    matchLoc.x += remapCenterX;
    matchLoc.y += remapCenterY;
    
    /// display located area
    if(debug)
    {
        //rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(255), 2, 8, 0 );
        rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(255), 2, 8, 0 );
    }
    
    // update by half of the template's width and height for actual center point
    matchLoc.x += templ.cols/2;
    matchLoc.y += templ.rows/2;
    
    //rectangle(img_display, Point(matchLoc.x - 3, matchLoc.y - 3), Point(matchLoc.x + 3, matchLoc.y + 3), Scalar::all(255));
    
    if(first || tmpdebug)
    {
        first = false;
        imwrite("imgDisplay.jpg", img_display);
        imwrite("scanImage.jpg", scanImage);
        imwrite("inputGrad.jpg", inputGrad);
        cout << "wrote debug files" << endl;
        tmpdebug = false;
    }
    
    if(debug)
    {
        cout << "writing debug pics" << endl;
        imwrite(miniFilePreamble + "image" + format(frameCount) + ".jpeg", img_display);
        imwrite(miniFilePreamble + "gradi" + format(frameCount) + ".jpeg", inputGrad);
        //processedOut.write(img_display);
        //gradientOut.write(inputGrad);
    }
    
    *cx = matchLoc.x;
    *cy = matchLoc.y;
    
    //cout << "center: " << *cx << ", " << *cy << endl;
    
    return;
}

static void onMouse(int event, int x, int y, int, void*)
{
    if(event == EVENT_LBUTTONDOWN)
    {
        if(!tracking && ( (x > leftBlack && x < leftViewNormalCamera) || (x > rightViewNormalCamera && x < rightBlack) ))
        {
            //cout << "X: " << x << "  Y: " << y << endl;
            tracking = true;
            setFrameRate(currentTrackingFrameRate);
            
            if(panoramaDegrees < 360)
            {
                if(rightBlack - x < trackingSliceWidth)
                {
                    x = rightBlack - trackingSliceWidth;
                }
                else if(x - leftBlack < trackingSliceWidth)
                {
                    x = leftBlack + trackingSliceWidth;
                }
            }
            
            commandsGiven = 0;
            degreesFromCenter = (x - (unwarpedW/2))*360/unwarpedW;
            //cout << "offset: " << degreesFromCenter << endl;
            heuristicTurns = abs((int)degreesFromCenter / 3.5);
            
            if(x >= trackingSliceWidth && x <= unwarpedW - trackingSliceWidth - 1)
            {
                // we can just grab the area of the slice normally
                cout << "normal template condition." << endl;
                Rect area = Rect(x - trackingSliceWidth, 0, 2 * trackingSliceWidth, unwarpedH);
                
                trackedImage = thePanorama(area).clone();
                cvtColor(trackedImage, trackedImage, COLOR_RGB2GRAY);
                
                //if(debug)
                {
                    imwrite("trackedImage.jpg", trackedImage);
                }
            }
            else
            {
                // we'll need to stitch together two sides of the panorama to get the template image
                cout << "stitching template condition. " << x << ", " << y << endl;
                Rect arealeft;
                Rect arearight;
                if(x < trackingSliceWidth)
                {
                    arealeft = Rect(unwarpedW + (x - trackingSliceWidth), 0, trackingSliceWidth - x, unwarpedH);
                    arearight = Rect(0, 0, x + trackingSliceWidth, unwarpedH);
                }
                else
                {
                    arealeft = Rect( x - trackingSliceWidth, 0, unwarpedW - (x - trackingSliceWidth), unwarpedH);
                    arearight = Rect(0, 0, (2 * trackingSliceWidth) - (unwarpedW - (x - trackingSliceWidth)), unwarpedH);
                }
                
                // make mats, append them
                Mat tmpL = thePanorama(arealeft).clone();
                Mat tmpR = thePanorama(arearight).clone();
                //cout << tmpL.cols << ", " << tmpL.rows << "   " << tmpR.cols << ", " << tmpR.rows << endl;
                //cout << trackedImage.cols << ", " << trackedImage.rows << endl;
                
                //trackedImage.deallocate();
                //cout << "trackedImage type: " << trackedImage.type() << endl;
                if(trackedImage.type() != 16) cvtColor(trackedImage, trackedImage, CV_GRAY2BGR);
                //trackedImage.convertTo(trackedImage, 16);
                //cout << "trackedImage type: " << trackedImage.type() << endl;
                
                tmpL.copyTo(trackedImage(Rect(0, 0, tmpL.cols, tmpL.rows)));
                tmpR.copyTo(trackedImage(Rect(tmpL.cols, 0, tmpR.cols, tmpR.rows)));
                
                cvtColor(trackedImage, trackedImage, COLOR_RGB2GRAY);
                
                if(debug)
                {
                    imwrite("trackedImage.jpg", trackedImage);
                }
            }
            if(debug)
            {
                cout << "trackedImage dimensions" << trackedImage.cols << ", " << trackedImage.rows << endl;
            }
            
            if(degreesFromCenter > 0)
            {
                leftX = unwarpedW/2 - 2*trackingSliceWidth;
                rightX = min(unwarpedW, x + 2*trackingSliceWidth);
                currentCommand = rightTurn;
            }
            else if(degreesFromCenter < 0)
            {
                currentCommand = leftTurn;
                leftX = max(0, x-2*trackingSliceWidth);
                rightX = unwarpedW/2 + 2*trackingSliceWidth;
            }
            else
            {
                tracking = false;
                cout << "zero degree turn" << endl;
            }
            
            // do stuff
            detector.detect( trackedImage, keypoints_track );
            extractor.compute( trackedImage, keypoints_track, descriptors_track );
            //cout << "keypoints in tracked image: " << keypoints_track.size() << " descriptors: " << descriptors_track.size() << endl;
        }
    }
}

void setFrameCount()
{
    while(!frame.data) // go until we find the first valid image file so we know where to start the framecount
    {
        frameCount += 25;
        cout << "looking for file: " << filepreamble + format(frameCount) + ".jpeg" << endl;
        frame = imread(filepreamble + format(frameCount) + ".jpeg");
    }
}

void setup()
{
    if(debug)
    {
        cout << "debug is on." << endl;
    }
    
    // run video grabber
    system("killall node");
    system("/usr/local/bin/node ../scripts/ZHIserver.js &");
    
    sleep(3);
    
    // run chrome
    system("killall -9 \"Google Chrome\"");
    system("/Applications/Google\\ Chrome.app/Contents/MacOS/Google\\ Chrome --app=http://drive.doublerobotics.com --user-data-dir=~/Library/Application\\ Support/Google/Chrome/Default/ --window-position=0,288 --window-size=2560,1127 &");
    
    sleep(4);
    
    // this is used if using imread() a ton
    frameCount = 0;
    frame = imread(filepreamble + format(frameCount) + ".jpeg");
    setFrameCount();
    
    setFrameRate(currentFrameRate);
    
    // window to show stream
    namedWindow(windowname, CV_WINDOW_AUTOSIZE);
    setMouseCallback(windowname, onMouse);
    moveWindow(windowname, 560, 0);
    
    // calculate the right edge of the left black section
    leftBlack = (unwarpedW/2) - ((unwarpedW*panoramaDegrees)/720.0);
    
    // calculate the left edge of the right black section
    rightBlack = (unwarpedW/2) + ((unwarpedW*panoramaDegrees)/720.0);
    
    // calculate black field of view bars
    int normalCameraOffset = unwarpedW*(degreesOfNormalCamera/720.0);
    leftViewNormalCamera = (unwarpedW/2.0) - normalCameraOffset;
    rightViewNormalCamera = (unwarpedW/2.0) + normalCameraOffset;
    
    // set training for tracking to be true
    if(panoramaDegrees > degreesOfNormalCamera) setUpTrackingTraining();
    
    // read in the template file
    templ = imread(templateFileName);
    if(!templ.data)
    {
        cout << "failed to read template file." << endl;
        exit(-1);
    }
}

bool isValid(Mat m)
{
    //cout << m.cols << " " << m.rows << endl;
    return m.cols == 1024 && m.rows == 768;
}

int main()
{
    setup();
    cx = -1; cy = -1;
    
    while(1337)
    {
        // read the current frame
        frame = imread(filepreamble + format(frameCount) + ".jpeg");
        actualCounter++;
        int past;
        
        if(!tracking)
        {
            past = rateList.back();
            rateList.pop_back();
        }
        else
        {
            past = trackingRateList.back();
            trackingRateList.pop_back();
        }
        
        if(isValid(frame))
        {
            // new framerate stuff
            if(!tracking)
            {
                rateList.push_front(0);
                if(past != 0)
                {
                    pastFailureCounter--;
                }
            }
            else
            {
                trackingRateList.push_front(0);
                if(past != 0)
                {
                    pastTrackingFailureCounter--;
                }
            }
            
            // 1) estimate the center
            estimateCenterSteve(&cx, &cy, frame);
            
            // 1.5) recalculate the center if our values are strange
            if(cx < frame.cols/5 || cx > 4*frame.cols/5)
            {
                cout << "unusual center detected. performing full center recalculation. " << cx << " " << cy << endl;
                cx = -1; cy = -1;
                estimateCenterSteve(&cx, &cy, frame);
            }
            
            // 2) unwarp the frame
            thePanorama = unwarpSimple(frame, &cx, &cy);
            
            frameCount++;
            failCount = 0;
        }
        else
        {
            // new framerate stuff
            if(!tracking)
            {
                rateList.push_front(1);
                if(past != 1)
                {
                    pastFailureCounter++;
                }
            }
            else
            {
                trackingRateList.push_front(1);
                if(past != 1)
                {
                    pastTrackingFailureCounter++;
                }
            }
            
            
            failCount++;
            
            if(failCount > 20)
            {
                cout << "couldn't keep up with image processing, performing reset" << endl;
                frameCount -= 30;
                setFrameCount();
                failCount = 0;
                
                if(tracking)
                {
                    currentTrackingFrameRate += 1;
                    setFrameRate(currentTrackingFrameRate);
                }
                else
                {
                    currentFrameRate += 1;
                    setFrameRate(currentFrameRate);
                }
                if(!trackingTraining) tracking = false;
            }
        }
        
        if(actualCounter % 25 == 0)
        {
            if(!tracking)
            {
                if(pastFailureCounter > 80)
                {
                    // we are waiting too often for new frames, should increase sample rate
                    currentFrameRate -= 1;
                    setFrameRate(currentFrameRate);
                }
                else if(pastFailureCounter < 50)
                {
                    // we are having trouble keeping up, decrease sample rate
                    currentFrameRate += 1;
                    setFrameRate(currentFrameRate);
                }
                //cout << pastFailureCounter << endl;
            }
            else
            {
                if(pastTrackingFailureCounter > 80)
                {
                    // we are waiting too often for new frames, should increase sample rate
                    currentTrackingFrameRate -= 1;
                    setFrameRate(currentTrackingFrameRate);
                }
                else if(pastTrackingFailureCounter < 50)
                {
                    // we are having trouble keeping up, decrease sample rate
                    currentTrackingFrameRate += 1;
                    setFrameRate(currentTrackingFrameRate);
                }
                //cout << pastTrackingFailureCounter << endl;
            }
        }
        
        if(tracking)
        {
            if(actualCounter%3 == 0)
            {
                //cout << "rightX: " << rightX << "  leftX: " << leftX << endl;
                Rect area = Rect(leftX, 0, rightX-leftX, unwarpedH);
                Mat centerOfPanorama = thePanorama(area).clone();
                
                // 1) detect keypoints
                detector.detect(centerOfPanorama, keypoints_pano);
                
                // 2) calculate descriptors (feature vectors)
                extractor.compute(centerOfPanorama, keypoints_pano, descriptors_pano);
                
                if(keypoints_pano.size() == 0 || keypoints_track.size() < 5)
                {
                    if(!trackingTraining)
                    {
                        cout << "this is probably going to cause issues:: " <<keypoints_pano.size() << " " << keypoints_track.size() << endl;
                        tracking = false;
                        continue;
                    }
                }
                
                // 3) matching descriptor vectors using flann matcher
                matcher.match( descriptors_pano, descriptors_track, matches );
                
                double min_dist = 100;
                
                // calculation of max and min distances between keypoints
                for( int i = 0; i < descriptors_pano.rows; i++ )
                {
                    double dist = matches[i].distance;
                    if( dist < min_dist ) min_dist = dist;
                }
                //cout << "min dist: " << min_dist << endl;
                
                //printf("-- Min dist : %f \n", min_dist );
                
                //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
                //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
                //-- small)
                //-- PS.- radiusMatch can also be used here.
                std::vector< DMatch > good_matches;
                
                for( int i = 0; i < descriptors_pano.rows; i++ )
                {
                    if(matches[i].distance <= max(2 * min_dist, 0.02))
                    {
                        good_matches.push_back( matches[i]);
                    }
                }
                
                // draw only "good" matches
                //Mat img_matches;
                //drawMatches( centerOfPanorama, keypoints_1, trackedImage, keypoints_2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                
                xAverage = 0.0;
                for(int i = 0; i < good_matches.size(); i++)
                {
                    //cout << keypoints_1[i].pt << endl;
                    //cout << good_matches[i].queryIdx << endl; // i think this is it
                    //cout << keypoints_1[good_matches[i].queryIdx].pt << endl;
                    xAverage += keypoints_pano[good_matches[i].queryIdx].pt.x;
                }
                xAverage /= good_matches.size();
                xAverage += leftX; // translate back to the full panorama's coord system
                //cout << "xAverage: " << xAverage << endl;
                
                // todo::: see if we can update to restrict the search space
                /*if(degreesFromCenter < 0)
                 {
                 //leftX = max(0, max(leftX, (int)(xAverage - trackingSliceWidth)));
                 cout << "left" << endl;
                 }
                 else
                 {
                 //rightX = min(unwarpedW, min(rightX, (int) (xAverage + trackingSliceWidth)));
                 cout << "right" << endl;
                 }*/
                
                rectangle(thePanorama, Point(xAverage - trackingSliceWidth, 0), Point(xAverage + trackingSliceWidth, unwarpedH), Scalar::all(255));
                
                //rectangle(thePanorama, Point(leftX, 0), Point(rightX, unwarpedH), Scalar(255,0,0));
                
                
                /*for( int i = 0; i < (int)good_matches.size(); i++ )
                 {
                 printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
                 }*/
                
                //-- Show detected matches
                //imwrite("matchesFile.jpg", img_matches);
                
                //cout << "distance from xAverage to center: " << abs(xAverage - (unwarpedW/2)) << " thresh: " << trackingThresh <<endl;
                
                // TODO:: heuristics are off!
                if((xAverage - (unwarpedW/2) < trackingThresh && (unwarpedW/2) - xAverage < trackingThresh) || commandsGiven > min(45, heuristicTurns))
                {
                    if(!trackingTraining)
                    {
                        trackingTimesUnderThresh++;
                        if(trackingTimesUnderThresh > 0)
                        {
                            tracking = false; // we have turned toward the object!
                            setFrameRate(currentFrameRate);
                        }
                        cout << "found it! " << commandsGiven << " commands used, predicted: " << heuristicTurns << endl;
                    }
                }
                else
                {
                    trackingTimesUnderThresh = 0;
                    
                    // issue a turn command
                    if(!trackingTraining)
                    {
                        system((currentCommand + " &").c_str());
                    }
                    commandsGiven++;
                }
            }
            else
            {
                xAverage += currentCommand==rightTurn ? -1 : 1;
                //rectangle(thePanorama, Point(xAverage - trackingSliceWidth, 0), Point(xAverage + trackingSliceWidth, unwarpedH), Scalar(0, 255, 0));
                //rectangle(thePanorama, Point(leftX, 0), Point(rightX, unwarpedH), Scalar(255,0,0));
            }
            if(trackingTraining)
            {
                trackingTrainingIterations++;
                if (trackingTrainingIterations > 500)
                {
                    trackingTraining = false;
                    tracking = false;
                    cout << "finished training" << endl;
                }
                else
                {
                    tracking = true;
                }
                // draw a visual that we are still training
                double percent = trackingTrainingIterations/500.0;
                int xVal = percent * unwarpedW;
                rectangle(thePanorama, Point(0,0), Point(unwarpedW,unwarpedH), Scalar(0,0,0), CV_FILLED, 8, 0);
                rectangle(thePanorama, Point(0,0), Point(xVal, unwarpedH), Scalar(255,0,0), CV_FILLED, 8, 0);
            }
        }
        
        // draw black field of view bars
        if(!trackingTraining)
        {
            line(thePanorama, Point(leftViewNormalCamera, 0), Point(leftViewNormalCamera, unwarpedH), Scalar::all(0), 2, 8, 0);
            line(thePanorama, Point(rightViewNormalCamera, 0), Point(rightViewNormalCamera, unwarpedH), Scalar::all(0), 2, 8, 0);
            
            
            // blacken out the unwanted sections of panorama
            rectangle(thePanorama, Point(0,0), Point(leftBlack, unwarpedH), Scalar(0,0,0), CV_FILLED, 8, 0);
            rectangle(thePanorama, Point(rightBlack,0), Point(unwarpedW,unwarpedH), Scalar(0,0,0), CV_FILLED, 8, 0);
        }
        
        
        
        if(thePanorama.rows > 0 && thePanorama.cols > 0) imshow(windowname, thePanorama);
        
        // if the key pressed by user is esc (ascii 27) then break out of the loop
        if(waitKey(1)==27)
            break;
        
    }
    cvDestroyAllWindows();
}