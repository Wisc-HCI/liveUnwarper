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
#include <png.h>
using namespace std;
using namespace cv;


//###########################################################
int panoramaDegrees = 360        ; // 360, 180, or 40 !
//###########################################################




// framerate stuff
int currentFrameRate = 180;//180;//125;
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
glutWindow win2;

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

GLuint png_texture_load(const char * file_name, int * width, int * height)
{
    png_byte header[8];
    
    FILE *fp = fopen(file_name, "rb");
    if (fp == 0)
    {
        perror(file_name);
        return 0;
    }
    
    // read the header
    fread(header, 1, 8, fp);
    
    if (png_sig_cmp(header, 0, 8))
    {
        fprintf(stderr, "error: %s is not a PNG.\n", file_name);
        fclose(fp);
        return 0;
    }
    
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
    {
        fprintf(stderr, "error: png_create_read_struct returned 0.\n");
        fclose(fp);
        return 0;
    }
    
    // create png info struct
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        fprintf(stderr, "error: png_create_info_struct returned 0.\n");
        png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
        fclose(fp);
        return 0;
    }
    
    // create png info struct
    png_infop end_info = png_create_info_struct(png_ptr);
    if (!end_info)
    {
        fprintf(stderr, "error: png_create_info_struct returned 0.\n");
        png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp) NULL);
        fclose(fp);
        return 0;
    }
    
    // the code in this if statement gets called if libpng encounters an error
    if (setjmp(png_jmpbuf(png_ptr))) {
        fprintf(stderr, "error from libpng\n");
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        fclose(fp);
        return 0;
    }
    
    // init png reading
    png_init_io(png_ptr, fp);
    
    // let libpng know you already read the first 8 bytes
    png_set_sig_bytes(png_ptr, 8);
    
    // read all the info up to the image data
    png_read_info(png_ptr, info_ptr);
    
    // variables to pass to get info
    int bit_depth, color_type;
    png_uint_32 temp_width, temp_height;
    
    // get info about png
    png_get_IHDR(png_ptr, info_ptr, &temp_width, &temp_height, &bit_depth, &color_type,
                 NULL, NULL, NULL);
    
    if (width){ *width = temp_width; }
    if (height){ *height = temp_height; }
    
    // Update the png info struct.
    png_read_update_info(png_ptr, info_ptr);
    
    // Row size in bytes.
    int rowbytes = png_get_rowbytes(png_ptr, info_ptr);
    
    // glTexImage2d requires rows to be 4-byte aligned
    rowbytes += 3 - ((rowbytes-1) % 4);
    
    // Allocate the image_data as a big block, to be given to opengl
    png_byte * image_data;
    image_data = (unsigned char *)(malloc(rowbytes * temp_height * sizeof(png_byte)+15));
    if (image_data == NULL)
    {
        fprintf(stderr, "error: could not allocate memory for PNG image data\n");
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        fclose(fp);
        return 0;
    }
    
    // row_pointers is for pointing to image_data for reading the png with libpng
    png_bytep * row_pointers = (unsigned char **)malloc(temp_height * sizeof(png_bytep));
    if (row_pointers == NULL)
    {
        fprintf(stderr, "error: could not allocate memory for PNG row pointers\n");
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        free(image_data);
        fclose(fp);
        return 0;
    }
    
    // set the individual row_pointers to point at the correct offsets of image_data
    int i;
    for (i = 0; i < temp_height; i++)
    {
        row_pointers[temp_height - 1 - i] = image_data + i * rowbytes;
    }
    
    // read the png into image_data through row_pointers
    png_read_image(png_ptr, row_pointers);
    
    // Generate the OpenGL texture object
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, temp_width, temp_height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    
    // clean up
    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
    free(image_data);
    free(row_pointers);
    fclose(fp);
    return texture;
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
    // TODO ::: uncomment below for actual running, this kills chrome and node
    // run video grabber
    system("killall node");
    system("/usr/local/bin/node ../scripts/ZHIserver.js &");
    
    sleep(3);
    
    // run chrome
    system("killall -9 \"Google Chrome\"");
    //system("/Applications/Google\\ Chrome.app/Contents/MacOS/Google\\ Chrome --app=http://drive.doublerobotics.com --user-data-dir=~/Library/Application\\ Support/Google/Chrome/Default/ --window-position=0,288 --window-size=2560,1127 &");
    system("/Applications/Google\\ Chrome.app/Contents/MacOS/Google\\ Chrome --app=http://drive.doublerobotics.com --user-data-dir=~/Library/Application\\ Support/Google/Chrome/Default/ --window-position=607,24 --window-size=1346,1415 &");
    
    sleep(4);
    
    // this is used if using imread() a ton
    frameCount = -10;
    frame = imread(filepreamble + format(frameCount) + ".jpeg");
    setFrameCount();
    cout << "just set frame count!" << endl;
    setFrameCount();
    cout << "set it again!" << endl;
    
    setFrameRate(currentFrameRate);
    
    // window to show stream
    /*
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
    */
    // read in the template file
    templ = imread(templateFileName);
    if(!templ.data)
    {
        cout << "failed to read template file." << endl;
        exit(-1);
    }
}

float degreesOfRobotFOV = 40.0;
float x = 3.7;
float y = 3.0;//5.0;
float zDist = 0;
float labelHeight = 1.0;

void displayLeft()
{
    GLfloat aspect = (GLfloat) win.width / win.height;
	gluPerspective(win.field_of_view_angle, aspect, win.z_near, win.z_far);		// set up a perspective projection matrix
    
    gluLookAt(0.0, 0.0, 5.00,  /* eye is at (0,0,5) */
              0.0, 0.0, 0.0,      /* center is at (0,0,0) */
              0.0, 1.0, 0.0);     /* up is in positive Y direction */
    
    glMatrixMode(GL_PROJECTION);												// select projection matrix
    
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		     // Clear Screen and Depth Buffer
	glLoadIdentity();
    
    // Convert to texture
	GLuint tex = matToTexture(thePanorama, GL_NEAREST, GL_NEAREST, GL_CLAMP, 33);
    
    // Bind texture
	glBindTexture(GL_TEXTURE_2D, tex);
    
    glBegin(GL_QUAD_STRIP);
    
    float leftXCoord = 1-(((panoramaDegrees/2.0)/360.0) + .5);
    float rightXCoord = (-degreesOfRobotFOV/720.0 + .5);
    
    //cout << leftXCoord << " " << rightXCoord << endl;
    
    glTexCoord2f(leftXCoord,0.0);
    glVertex3f(-x,y,zDist);
    glTexCoord2f(leftXCoord,1.0);
    glVertex3f(-x,-y,zDist);
    // center panel
    //glTexCoord2f(0.0,1.0);
    glTexCoord2f(rightXCoord, 0.0);
    glVertex3f(x,y,zDist);
    glTexCoord2f(rightXCoord, 1.0);
    glVertex3f(x,-y,zDist);
    
    glEnd();
    
    //namedWindow(windowname, CV_WINDOW_AUTOSIZE);
    //moveWindow(windowname, 560, 0);
    //imshow(windowname, rightMat);
    
    GLuint leftTex = matToTexture(leftMat, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_BORDER, 31);
    
    // left label display
    glBindTexture(GL_TEXTURE_2D, leftTex);
    glBegin(GL_QUADS);
    
    glTexCoord2f(0.0,0.0);
    glVertex3f(-1.25,-y-.4,zDist);
    
    glTexCoord2f(1.0,0.0);
    glVertex3f(1.25,-y-.4,zDist);
    glTexCoord2f(1.0,1.0);
    glVertex3f(1.25,-y-.4-labelHeight,zDist);
    
    glTexCoord2f(0.0,1.0);
    glVertex3f(-1.25,-y-.4-labelHeight,zDist);
    
    
    glEnd();
    
    // free the texture memory
    glDeleteTextures(1, &tex);
    glDeleteTextures(1, &leftTex);
    
    glutSwapBuffers();
}

void displayRight()
{
    GLfloat aspect = (GLfloat) win.width / win.height;
	gluPerspective(win.field_of_view_angle, aspect, win.z_near, win.z_far);		// set up a perspective projection matrix
    
    gluLookAt(0.0, 0.0, 5.00,  /* eye is at (0,0,5) */
              0.0, 0.0, 0.0,      /* center is at (0,0,0) */
              0.0, 1.0, 0.0);     /* up is in positive Y direction */
    
    glMatrixMode(GL_PROJECTION);												// select projection matrix
    
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		     // Clear Screen and Depth Buffer
	glLoadIdentity();
    
    // Convert to texture
	GLuint tex = matToTexture(thePanorama, GL_NEAREST, GL_NEAREST, GL_CLAMP, 39);
    
    // Bind texture
	glBindTexture(GL_TEXTURE_2D, tex);
    
    glBegin(GL_QUAD_STRIP);
    
    float leftXCoord = (degreesOfRobotFOV/720.0 + .5);
    float rightXCoord = ((panoramaDegrees/2.0)/360.0) + .5;
    
    //cout << leftXCoord << " " << rightXCoord << endl;
    
    glTexCoord2f(leftXCoord,0.0);
    glVertex3f(-x,y,zDist);
    glTexCoord2f(leftXCoord,1.0);
    glVertex3f(-x,-y,zDist);
    // center panel
    //glTexCoord2f(0.0,1.0);
    glTexCoord2f(rightXCoord, 0.0);
    glVertex3f(x,y,zDist);
    glTexCoord2f(rightXCoord, 1.0);
    glVertex3f(x,-y,zDist);
    
    glEnd();
    
    GLuint rightTex = matToTexture(rightMat, GL_NEAREST, GL_NEAREST, GL_CLAMP, 32);
    
    // left label display
    glBindTexture(GL_TEXTURE_2D, rightTex);
    glBegin(GL_QUADS);
    
    glTexCoord2f(1.0,0.0);
    glVertex3f(1.25,-y-.4,zDist);
    
    glTexCoord2f(0.0,0.0);
    glVertex3f(-1.25,-y-.4,zDist);
    glTexCoord2f(0.0,1.0);
    glVertex3f(-1.25,-y-.4-labelHeight,zDist);
    
    glTexCoord2f(1.0,1.0);
    glVertex3f(1.25,-y-.4-labelHeight,zDist);
    
    
    glEnd();
    
    // free the texture memory
    glDeleteTextures(1, &tex);
    glDeleteTextures(1, &rightTex);
    
    glutSwapBuffers();
}

#define KEY_ESCAPE 27

void keyboard ( unsigned char key, int mousePositionX, int mousePositionY )
{
    switch ( key )
    {
        case KEY_ESCAPE:
            exit ( 0 );
            break;
        default:
            break;
    }
}

void initialize()
{
    glEnable( GL_DEPTH_TEST );
    glEnable(GL_TEXTURE_2D);
    glDepthFunc( GL_LEQUAL );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );						// specify implementation-specific hints
	glClearColor(0.0, 0.0, 0.0, 1.0);											// specify clear values for the color buffers
}

bool isValid(Mat m)
{
    //cout << m.cols << " " << m.rows << endl;
    return m.cols == 1024 && m.rows == 768;
}

void refresher()
{
    // read the current frame
    frame = imread(filepreamble + format(frameCount) + ".jpeg");
    actualCounter++;
    
    int past = rateList.back();
    rateList.pop_back();
    
    //cout << "framecount " << frameCount << endl;
    
    if(isValid(frame))
    {
        //cout << "valid frame." << endl;
        // new framerate stuff
        rateList.push_front(0);
        if(past != 0)
        {
            pastFailureCounter--;
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
        glutSetWindow(win.id);
        glutPostRedisplay();
        glutSetWindow(win2.id);
        glutPostRedisplay();
    }
    else
    {
        //cout << "invalid frame." << endl;
        // new framerate stuff
        rateList.push_front(1);
        if(past != 1)
        {
            pastFailureCounter++;
        }
        
        failCount++;
        
        if(failCount > 80000)
        {
            cout << "couldn't keep up with image processing, performing reset" << endl;
            frameCount -= 30;
            setFrameCount();
            failCount = 0;
            
            currentFrameRate += 1;
            setFrameRate(currentFrameRate);
        }
    }
    /*
    if(actualCounter % 150 == 0)
    {
            cout << "past failure counter: " << pastFailureCounter << endl;
            if(pastFailureCounter > 80)
            {
                // we are waiting too often for new frames, should increase sample rate
                currentFrameRate -= 1;
                setFrameRate(currentFrameRate);
            }
            else if(pastFailureCounter < 60)
            {
                // we are having trouble keeping up, decrease sample rate
                currentFrameRate += 1;
                setFrameRate(currentFrameRate);
            }
            //cout << pastFailureCounter << endl;
            
            // we care about the CHANGE in the pastFailureCounter (if the change is
        
    }*/
    //if(thePanorama.rows > 0 && thePanorama.cols > 0) imshow(windowname, thePanorama);
    //cout << "ran refresher: " << isValid(thePanorama) << endl;
}


void setupOpenGL(int argc, char **argv)
{
    int offset = 45; // offset from top of screen
	// initialize and run program
	glutInit(&argc, argv);                                      // GLUT initialization
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );  // Display Mode
    
    win.width = 607;
    win.height = 1440 - offset; // make this roughly the height of the monitor
    win.title = "Double";
    win.field_of_view_angle = 120;
    win.z_near = 0.1f;
    win.z_far = 500.0f;
    
    glutInitWindowSize(win.width,win.height);					// set window size
    win.id = glutCreateWindow(win.title);								// create Window
    glutPositionWindow(0,offset);
    glutDisplayFunc(displayLeft);									// register Display Function
    glutKeyboardFunc(keyboard);                                // register Keyboard Handler
    
    initialize();
    
    win2.width = win.width;
    win2.height = win.height; // make this roughly the height of the monitor
    win2.title = "Double";
    win2.field_of_view_angle = win.field_of_view_angle;
    win2.z_near = win.z_near;
    win2.z_far = win.z_far;
    
    win2.id = glutCreateWindow(win2.title);
    glutPositionWindow(2560 - win2.width, offset); // 1440 is screen resolution x
    glutDisplayFunc(displayRight);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(refresher);
    
    initialize();
    
    int a = 0, b = 0;
    
    // load left and right textures
    //leftTex = png_texture_load(leftFile.c_str(), &a, &b);
    //rightTex = png_texture_load(rightFile.c_str(), &a, &b);
    
    
    leftMat = imread(leftFile);
    rightMat = imread(rightFile);
}



int main(int argc, char **argv)
{
    setup();
    cx = -1; cy = -1;
    setupOpenGL(argc, argv);
    cout << "about to start glutMainLoop" << endl;
    glutMainLoop();
    cout << "we shouldn't ever reach this! :)" << endl;
    return 1337;
}