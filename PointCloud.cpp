#include <Windows.h>
#include <Ole2.h>
#include <NuiApi.h>
#include<stdio.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <gl\glut.h>
#include <gl\GL.h>
#include "JpegLoader.h"

using namespace std;

#define width 640
#define height 480

void drawKinectData();
void saveToFile();

//OpenGl variables
GLubyte data_C[width*height*4],data_M[width*height*4];
USHORT data_D[width*height],raw_depth[width*height];
LONG colorCoordinates[width*height*2];

// Kinect variables
HANDLE depthStream,colorStream;//,NextSkeletonEvent = NULL;;
HRESULT hr;
Vector4 position[width*height];

INuiSensor* sensor;

void draw() {
   drawKinectData();
   glutSwapBuffers();
}

void execute() {
    glutMainLoop();
}

// to convert data obtained fro microsoft Kinect SDK
// format - 
// 0 ascii
// 1 binary_little_endian
// 2 binary_big_endian

// Saves the Depth Image (i,j,Z)
void PlyConvert(char *name,USHORT *depth_data,int resolution,int format)
{
	FILE *fp=fopen(name,"w");
	fprintf(fp,"ply\n");
	switch(format)
	{
	case 0:fprintf(fp,"format ascii 1.0\n");
		break;
	case 1:fprintf(fp,"format binary_little_endian 1.0\n");
		break;
	case 3:fprintf(fp,"format binary_big_endian 1.0\n");
		break;
	}
	fprintf(fp,"comment made by Vishesh Gupta\n");
	fprintf(fp,"comment This contains a Point Cloud\n");
	fprintf(fp,"element vertex %d\n",resolution);
	fprintf(fp,"property float x\n");
	fprintf(fp,"property float y\n");
	fprintf(fp,"property float z\n");
	fprintf(fp,"end_header\n");
	for(int y=0;y<height;y++)
	{
		int yindex = y*width;
		for(int x=0;x<width;x++)
		{
			int index= yindex + x;
			fprintf(fp,"%d %d %f\n",-1*x,y,((float)depth_data[index]));	// stores depth in file in mm...
		}
	}
	fclose(fp);
}

// Saves the Colored Point Cloud (x,y,z) & (r,g,b)
void PlyConvert(char *name,GLubyte *color,Vector4 *vertices,int resolution,int format)
{
	int count=0;

	for(int i=0;i<resolution;i++)
		if(vertices[i].z>0)
			count++;

	FILE *fp=fopen(name,"w");
	fprintf(fp,"ply\n");
	switch(format)
	{
	case 0:fprintf(fp,"format ascii 1.0\n");
		break;
	case 1:fprintf(fp,"format binary_little_endian 1.0\n");
		break;
	case 3:fprintf(fp,"format binary_big_endian 1.0\n");
		break;
	}
	fprintf(fp,"comment made by Vishesh Gupta\n");
	fprintf(fp,"comment This contains a Point Cloud\n");
	fprintf(fp,"element vertex %d\n",count);
	fprintf(fp,"property float x\n");
	fprintf(fp,"property float y\n");
	fprintf(fp,"property float z\n");
	fprintf(fp,"property uchar blue\n");
	fprintf(fp,"property uchar green\n");
	fprintf(fp,"property uchar red\n");
	fprintf(fp,"end_header\n");

	for(int i=0;i<resolution;i++)
	{
		if(vertices[i].z>0)
		{
			int index=i*4;
			fprintf(fp,"%f %f %f %u %u %u\n", vertices[i].x,vertices[i].y,vertices[i].z,color[index],color[index+1],color[index+2]);
		}
	}
	fclose(fp);
}

//Initialize the Kinect Sensor
bool initKinect() {
    // Get a working kinect sensor
    int numSensors;
    if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
    if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;
	
    // Initialize sensor
	sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR );//| NUI_INITIALIZE_FLAG_USES_SKELETON);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
        NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
        0,//NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE,									         // Image stream flags, e.g. near mode
        2,											 // Number of frames to buffer
        NULL,									 // Event handle
        &depthStream);

	if(sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480,0,2,NULL,&colorStream) < 0) return 0;
	
	//if(sensor->NuiSkeletonTrackingEnable(NextSkeletonEvent,0));
	return sensor;
}

//Retrieves the Depth and Color data coming from the Kinect Sensor into-
// dest_D : 16 bit Depth stream 
// dest_C : 8 bit color stream
void getKinectData(USHORT* dest_D,GLubyte* dest_C, USHORT *raw_depth)//,Vector4 *pos) 
{

    NUI_IMAGE_FRAME depthFrame,colorFrame;
    NUI_LOCKED_RECT LockedRect;
	//NUI_SKELETON_FRAME skeletonFrame;
	
	//for depth texture......................
	if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &depthFrame) < 0) return;
	INuiFrameTexture* textureD = depthFrame.pFrameTexture;
	textureD->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch != 0) 
	{
        const USHORT* curr = (const USHORT*) LockedRect.pBits;
        const USHORT* dataEnd = curr + (width*height);

        while (curr < dataEnd)
		{
            // Get depth in millimeters
            USHORT depth = NuiDepthPixelToDepth(*curr++);
			*raw_depth++  =depth;
			*dest_D++ = depth<<3;			
		}
		textureD->UnlockRect(0);
		sensor->NuiImageStreamReleaseFrame(depthStream, &depthFrame);
    }

	////for color texture.....................................
	if (sensor->NuiImageStreamGetNextFrame(colorStream, 0, &colorFrame) < 0) return;
	INuiFrameTexture* textureC = colorFrame.pFrameTexture;
	textureC->LockRect(0,&LockedRect,NULL,0);
	if (LockedRect.Pitch != 0) 
	{
		const BYTE* curr = (const BYTE*) LockedRect.pBits;
		const BYTE* dataEnd = curr + (width*height)*4;
		while (curr < dataEnd) {
			*dest_C++ = *curr++;
		}
		textureC->UnlockRect(0);
		sensor->NuiImageStreamReleaseFrame(colorStream, &colorFrame);
	}

	
}

void copyVector(Vector4 &dest,Vector4 &src)
{
	dest.x=src.x;		// kinect gives mirror images to correct them..........
	dest.y=src.y;
	dest.z=src.z;
	dest.w=src.w;
}

//Transforma the incoming Depth data to Positions in a 3d Euclidean Coordinate system with Kinect sensor as its Origin.
void MapDepthToSkeleton(USHORT *source,Vector4 *pos)
{
	for(int y=0;y<height;y++)
	{
		int yindex = y*width;
		for(int x=0;x<width;x++)
		{
			int index= yindex + x;
			// return x,y and z in meters.....
			copyVector(pos[index],NuiTransformDepthImageToSkeleton(
				x,
				y,
				source[index],
				NUI_IMAGE_RESOLUTION_640x480));
		}
	}
}

//Map the incoming color and depth stream from kinect with each other. 
void MapColorToDepth(USHORT *dsource, GLubyte *csource,GLubyte *cdest)
{
	sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		NUI_IMAGE_RESOLUTION_640x480,
		NUI_IMAGE_RESOLUTION_640x480,
		width*height,
		dsource,
		width*height*2,
		colorCoordinates);
	for(int i=0;i<width*height*4;i++)
			cdest[i]=0;

	for(int depthIndex=0;depthIndex<width*height;depthIndex++)
	{
		if(dsource[depthIndex]>0 && dsource[depthIndex]<=32000)//24000)//)		// 24000 for near mode; depth value shifted left by 3 bits
		{
			int mapIndex = depthIndex*2;
			int x=colorCoordinates[mapIndex];		// get the x coordinate in color image frame
			int y=colorCoordinates[mapIndex + 1];		// get the y coordinate in color image frame
		
			if(x>=0 && x<=640 && y>=0 && y<=480)
			{
				int colorIndex=(y*width*4 + x*4);		// get the index no in the color buffer where the color information corresponding to the
				for(int k=0;k<4;k++)					// X AND Y positions are stored.
					cdest[colorIndex]=csource[colorIndex++];
			}
		}
	}
}

//Calls the functions to save the data in .ply format 
void saveToFile()
{
	char name[15];
	cout<<"Enter the xyz PCD File Name \t";
	cin>>name;
	// to write the point cloud in x,y,z;
	PlyConvert(name,data_M,position,width*height,0);
	cout<<"\nWrite Complete x,y,z Point Cloud.....\n";

	// to write the point cloud in i,j,z; 
	cout<<"Enter the ijz PCD File Name \t";
	cin>>name;
	PlyConvert(name,raw_depth,width*height,0);
	cout<<"\nWrite complete i,j,depth point cloud....";
}

//Listen for mouse event while the application is Active
void mouse(GLint button, GLint state, GLint x, GLint y)
{
	if(button==GLUT_LEFT_BUTTON && state==GLUT_DOWN)
		saveToFile();
}

//Draw the Graphics for display on Screen
void drawKinectData()
{
	getKinectData(data_D,data_C,raw_depth);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	MapDepthToSkeleton(data_D,position);
	MapColorToDepth(data_D,data_C,data_M);
	
	glBegin(GL_POINTS);
		for(int i=0;i<height*width;i++)
			if(position[i].z>0)
			{
				int index=i*4;
				glColor3f((float)data_M[index+2]/255.0,(float)data_M[index+1]/255.0,(float)data_M[index]/255.0);	// color info stored in BGRA format......
				glVertex3f(position[i].x,position[i].y,position[i].z);
			}
    glEnd();
}

//Windowing and graphics system intialization
bool init(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(width,height);
	glutCreateWindow("Kinect Point Cloud");
	glutMouseFunc(mouse);
	glutDisplayFunc(draw);
    glutIdleFunc(draw);
    return true;
}

//OpenGL setup and Start of OpenGL main Loop
int main(int argc, char* argv[]) {
    if (!init(argc, argv)) return 1;
    if (!initKinect()) return 1;

	 // OpenGL setup
    glClearColor(0,0,0,0);
    glClearDepth(1.0f);
    
	// Camera setup
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1,1,-1,1,0.1,3);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	gluLookAt(0,0,0,0,0,1,0,1,0);			//camera positioned at origin, looking at +z direction.........
    // Main loop
    execute();
    return 0;
}