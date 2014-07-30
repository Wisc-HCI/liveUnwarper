//
//  main.cpp
//  liveUnwarper
//
//  Created by ------ on -/--/--.
//

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <list>
#include <unistd.h>
#include <GLUT/GLUT.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
using namespace std;
using namespace cv;


//###########################################################
int panoramaDegrees = 360        ; // 360, 180, or 40 !
//###########################################################




// framerate stuff
int currentFrameRate = 200;//180;//125;
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

// TODO tmp
string templateFileName = "/Users/hcilab/Documents/Steve/PanoramicStudy/liveUnwarper/liveUnwarper/template960.jpg";
//string templateFileName = "template960.jpg";


//int unwarpedW = 1300, unwarpedH = 240;
int unwarpedW = 1440, unwarpedH = 240;
//int unwarpedW = 1140, unwarpedH = 200;
//int unwarpedW = 770, unwarpedH = 180;

Mat thePanorama;
Mat theFront;

// TODO tmp
string frameRateCommand = "python /Users/hcilab/Documents/Steve/PanoramicStudy/liveUnwarper/scripts/frameRateChanger.py ";
//string frameRateCommand = "python ../scripts/frameRateChanger.py ";

const double PI = 3.1415926535897932384626;

string windowname = "Double";
int failCount = 0;

int degreesOfNormalCamera = 40;
int leftViewNormalCamera, rightViewNormalCamera;
Mat frame;
Mat templ;
Mat inputGrad;
int frameCount, maxFrameCount;
int frontFrameCount;

// unwarping parameters
int cx = -1, cy = -1;
double scaleX = 1.0, scaleY = 1.0;

float angularOffset = 0.0f; // angle (in radians) to start unwarping from

int scanSideDist = 60; // was 40
int remapCenterX;
int remapCenterY;

//string pathToVideoImages = "/Users/steven/Documents/madison/panoramic/steveB/tmp/";
//string pathToVideoImages = "/Users/steven/dev/liveUnwarper/scripts/tmp/";


//TODO ::: tmp
//string pathToVideoImages = "../scripts/tmp/";
string pathToVideoImages = "/Users/hcilab/Documents/Steve/PanoramicStudy/liveUnwarper/scripts/tmp/";

string filepreamble = pathToVideoImages + "unwarp/image";
string frontfilepreamble = pathToVideoImages + "front/image";
string miniFilePreamble = pathToVideoImages + "debug/";

typedef struct {
    int id;
    int width;
	int height;
	char* title;
    
	float field_of_view_angle;
	float z_near;
	float z_far;
} glutWindow;

glutWindow win;

string leftFile = "leftTex.jpg";
string rightFile = "rightTex.jpg";
Mat leftMat, rightMat;

// Function turn a cv::Mat into a texture, and return the texture ID as a GLuint for use
GLuint matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter, int flag)
{
	// Generate a number for our textureID's unique handle
	GLuint textureID;
	glGenTextures(1, &textureID);
    
    //TODO ::: testing
    textureID = flag;
    
	// Bind to our texture handle
	glBindTexture(GL_TEXTURE_2D, textureID);
    
	// Catch silly-mistake texture interpolation method for magnification
	if (magFilter == GL_LINEAR_MIPMAP_LINEAR  ||
	    magFilter == GL_LINEAR_MIPMAP_NEAREST ||
	    magFilter == GL_NEAREST_MIPMAP_LINEAR ||
	    magFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
		magFilter = GL_LINEAR;
	}
    
	// Set texture interpolation methods for minification and magnification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
    
	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);
    
	// Set incoming texture format to:
	// GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
	// GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
	// Work out other mappings as required ( there's a list in comments in main() )
	GLenum inputColourFormat = GL_BGR;
	if (mat.channels() == 1)
	{
		inputColourFormat = GL_LUMINANCE;
	}
    
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
	// Create the texture
	glTexImage2D(GL_TEXTURE_2D,     // Type of texture
	             0,                 // Pyramid level (for mip-mapping) - 0 is the top level
	             GL_RGB,            // Internal colour format to convert to
	             mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
	             mat.rows,          // Image height i.e. 480 for Kinect in standard mode
	             0,                 // Border width in pixels (can either be 1 or 0)
	             inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
	             GL_UNSIGNED_BYTE,  // Image data type
	             mat.ptr());        // The actual image data itself
    
	return textureID;
}

string toString ( int Number )
{
    ostringstream ss;
    ss << Number;
    return ss.str();
}

void setFrameRate(int rate, int port)
{
    // sets delay for sampling of image stream
    system((frameRateCommand + toString(rate) + " " + toString(port) + " &").c_str());
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
    
    // update by half of the template's width and height for actual center point
    matchLoc.x += templ.cols/2;
    matchLoc.y += templ.rows/2;
    
    *cx = matchLoc.x;
    *cy = matchLoc.y;
    
    //cout << "center: " << *cx << ", " << *cy << endl;
    
    return;
}

float degreesOfRobotFOV = 48.0;
float y = 5.0;//3.0;
float zDist = 0;

float periphEdgeX = 15.86;
float frontWidth = 5.4;
float frontHeight = 8.4;

void displayAll()
{
    GLfloat aspect = (GLfloat) win.width / win.height;
	gluPerspective(win.field_of_view_angle, aspect, win.z_near, win.z_far);
    
    gluLookAt(0.0, 0.0, 5.00,     // eye is at (0,0,5)
              0.0, 0.0, 0.0,      // center is at (0,0,0)
              0.0, 1.0, 0.0);     // up is in positive Y direction
    
    glMatrixMode(GL_PROJECTION);
    
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
    
    
    GLuint frontTex = matToTexture(theFront, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_BORDER, 31);
    
    // front display
    glBindTexture(GL_TEXTURE_2D, frontTex);
    glBegin(GL_QUADS);
    
    glTexCoord2f(0.0,0.0);
    glVertex3f(-frontWidth,frontHeight,zDist);
    
    glTexCoord2f(1.0,0.0);
    glVertex3f(frontWidth,frontHeight,zDist);
    glTexCoord2f(1.0,1.0);
    glVertex3f(frontWidth,-frontHeight,zDist);
    
    glTexCoord2f(0.0,1.0);
    glVertex3f(-frontWidth,-frontHeight,zDist);
    
    glEnd();
    
    
    // convert to texture
	GLuint tex = matToTexture(thePanorama, GL_NEAREST, GL_NEAREST, GL_CLAMP, 33);
    
    if(panoramaDegrees != 40)
    {
    // bind texture
	glBindTexture(GL_TEXTURE_2D, tex);
    
    glBegin(GL_QUAD_STRIP);
    
    float leftXCoord = 1-(((panoramaDegrees/2.0)/360.0) + .5);
    float rightXCoord = (-degreesOfRobotFOV/720.0 + .5);
    
    //cout << leftXCoord << " " << rightXCoord << endl;
    
    glTexCoord2f(leftXCoord,0.0);
    glVertex3f(-periphEdgeX,y,zDist);
    glTexCoord2f(leftXCoord,1.0);
    glVertex3f(-periphEdgeX,-y,zDist);
    glTexCoord2f(rightXCoord, 0.0);
    glVertex3f(-frontWidth,y,zDist);
    glTexCoord2f(rightXCoord, 1.0);
    glVertex3f(-frontWidth,-y,zDist);
    
    glEnd();
    
    glBegin(GL_QUAD_STRIP);
    
    leftXCoord = (degreesOfRobotFOV/720.0 + .5);
    rightXCoord = ((panoramaDegrees/2.0)/360.0) + .5;
    
    //cout << leftXCoord << " " << rightXCoord << endl;
    
    glTexCoord2f(leftXCoord,0.0);
    glVertex3f(frontWidth,y,zDist);
    glTexCoord2f(leftXCoord,1.0);
    glVertex3f(frontWidth,-y,zDist);
    glTexCoord2f(rightXCoord, 0.0);
    glVertex3f(periphEdgeX,y,zDist);
    glTexCoord2f(rightXCoord, 1.0);
    glVertex3f(periphEdgeX,-y,zDist);
    
    glEnd();
    }
    // free the texture memory
    glDeleteTextures(1, &tex);
    glDeleteTextures(1, &frontTex);
    
    glutSwapBuffers();
}

void setFrameCount()
{
    while(!frame.data) // go until we find the first valid image file so we know where to start the framecount
    {
        frameCount += 25;
        cout << "looking for file: " << filepreamble + format(frameCount) + ".jpeg" << endl;
        frame = imread(filepreamble + format(frameCount) + ".jpeg");
    }
    frameCount += 10;
}

void setFrontFrameCount()
{
    while(!theFront.data) // go until we find the first valid image file so we know where to start the framecount
    {
        frontFrameCount += 25;
        cout << "looking for file: " << frontfilepreamble + format(frontFrameCount) + ".jpeg" << endl;
        theFront = imread(frontfilepreamble + format(frontFrameCount) + ".jpeg");
    }
    frontFrameCount += 10;
}

void setup()
{
    if(!(panoramaDegrees == 360 || panoramaDegrees == 180 || panoramaDegrees == 40))
    {
        // invalid condition
        cout << "please select a valid study condition: 360, 180, or 40" << endl;
        exit(0);
    }
    else
    {
        if(panoramaDegrees == 180)
        {
            periphEdgeX = ((periphEdgeX - frontWidth)/2.0) + frontWidth;
        }
    }
    // TODO ::: uncomment below for actual running, this kills chrome and node
    // run video grabber
    system("killall node");
    //system("/usr/local/bin/node ../scripts/panoServer.js &");
    //system("/usr/local/bin/node ../scripts/frontServer.js &");
    
    system("/usr/local/bin/node /Users/hcilab/Documents/Steve/PanoramicStudy/liveUnwarper/scripts/panoServer.js &");
    system("/usr/local/bin/node /Users/hcilab/Documents/Steve/PanoramicStudy/liveUnwarper/scripts/frontServer.js &");
    
    
    
    sleep(3);
    
    // run chrome
    system("killall -9 \"Google Chrome\"");
    system("/Applications/Google\\ Chrome.app/Contents/MacOS/Google\\ Chrome --app=http://drive.doublerobotics.com --user-data-dir=~/Library/Application\\ Support/Google/Chrome/Default/ --window-position=0,1300 --window-size=2560,140 &");
    //system("/Applications/Google\\ Chrome.app/Contents/MacOS/Google\\ Chrome --app=http://drive.doublerobotics.com --user-data-dir=~/Library/Application\\ Support/Google/Chrome/Default/ --window-position=0,288 --window-size=2560,1127 &");
    //system("/Applications/Google\\ Chrome.app/Contents/MacOS/Google\\ Chrome --app=http://drive.doublerobotics.com --user-data-dir=~/Library/Application\\ Support/Google/Chrome/Default/ --window-position=607,24 --window-size=1346,1415 &");
    
    sleep(4);
    
    frameCount = -10, frontFrameCount = -10;
    frame = imread(filepreamble + format(frameCount) + ".jpeg");
    setFrameCount();
    cout << "just set frame count! " << frameCount << endl;
    setFrameCount();
    cout << "set it again! " << frameCount << endl;
    
    theFront = imread(frontfilepreamble + format(frontFrameCount) + ".jpeg");
    setFrontFrameCount();
    cout << "just set front frame count! " << frontFrameCount << endl;
    setFrontFrameCount();
    cout << "set it again! " << frontFrameCount << endl;
    
    setFrameRate(currentFrameRate, 9001);
    setFrameRate(currentFrameRate - 30, 9002);
    
    // read in the template file
    templ = imread(templateFileName);
    if(!templ.data)
    {
        cout << "failed to read template file." << endl;
        exit(-1);
    }
}

#define KEY_ESCAPE 27

void keyboard (unsigned char key, int mousePositionX, int mousePositionY)
{
    switch (key)
    {
        case KEY_ESCAPE:
            exit(0);
            break;
        default:
            break;
    }
}

bool isValid(Mat m)
{
    //cout << m.cols << " " << m.rows << endl;
    return m.cols == 1024 && m.rows == 768;
}

bool isValidFront(Mat m)
{
    // TODO ::: this may change
    return m.cols == 768 && m.rows == 1024;
}

void refresher()
{
    bool postRedisplay = false;
    
    // read the current pano frame
    frame = imread(filepreamble + format(frameCount) + ".jpeg");
    // read the current front frame
    Mat tmpfront = imread(frontfilepreamble + format(frontFrameCount) + ".jpeg");
    
    actualCounter++;
    
    //int past = rateList.back();
    //rateList.pop_back();
    
    //cout << "framecount " << frameCount << endl;
    //cout << "frontframecount " << frontFrameCount << endl;
    
    
    if(isValidFront(tmpfront))
    {
        theFront = tmpfront;
        frontFrameCount++;
        postRedisplay = true;
    }
    
    if(isValid(frame))
    {
        //cout << "valid frame." << endl;
        // new framerate stuff
        //rateList.push_front(0);
        //if(past != 0)
        //{
        //    pastFailureCounter--;
        //}
        
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
        //failCount = 0;
        postRedisplay = true;
    }
    
    if(postRedisplay)
    {
        glutPostRedisplay();
    }
}

void initialize()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glClearColor(0.0, 0.0, 0.0, 1.0);
}

void setupOpenGL(int argc, char **argv)
{
    int offset = 45; // offset from top of screen
    
	// initialize and run program
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
    
    win.width = 2560;
    win.height = 1440 - offset; // make this roughly the height of the monitor
    win.title = "Double";
    win.field_of_view_angle = 120;
    win.z_near = 0.1f;
    win.z_far = 500.0f;
    
    glutInitWindowSize(win.width,win.height);
    win.id = glutCreateWindow(win.title);
    glutPositionWindow(0,offset);
    glutDisplayFunc(displayAll);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(refresher);
    
    initialize();
    
    leftMat = imread(leftFile);
    rightMat = imread(rightFile);
}



int main(int argc, char **argv)
{
    setupOpenGL(argc, argv);
    setup();
    cx = -1; cy = -1;
    
    cout << "about to start glutMainLoop" << endl;
    glutMainLoop();
    cout << "we shouldn't ever reach this! :)" << endl;
    return 1337;
}