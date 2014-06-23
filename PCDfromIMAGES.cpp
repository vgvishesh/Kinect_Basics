#include<iostream>
#include<stdio.h>
#include<glm\glm.hpp>
#include <glm/gtc/type_ptr.hpp>	
#include<string>
#include<ctype.h>

using namespace std;

// to convert data obtained fro microsoft Kinect SDK
// format - 
// 0 ascii
// 1 binary_little_endian
// 2 binary_big_endian

//	// kinect transformation matrix for (i,j,k) -> (x,y,z)
//	// a b c d 			i
//	// e f g h 	x		j
//	// 0 0 0 1			k
//	//					1
//	//c= -0.00007002
//	//d= 0
//	//a= m * k/1000
//	//b= 0
//	//g= 0.000052515
//	//h= 0
//	//e= 0
//	//f= -m * k/1000
//	//m= 0.000219

//uses the transformation matrix to transform <i,j,k> to <x,y,z>
//	x`		m  0  c   0				i
//	y`	=	0 -m  g   0		*   	j
//	w		0  0  1/k 0				1000
//									1
//								
//	m= 0.000219
//	c= -0.00007002
//	g= 0.000052515
//
//	x=x`/w
//	y=y`/w
//	z=k/0.000125
void Transform2Dto3D(char *ip1,char *ip2,char *op,int format)
{
	double *Data,position;
	long NumVertexD;
	int Depth;
	char str[100],*substr="element",NumVertexC[15],*end_header="end_header",a[10],raw_depth[30];
	glm::dvec4 PointCloudVertex,TransformedVertex;
	FILE *inc=fopen(ip1,"r"),*ind=fopen(ip2,"r"),*out=fopen(op,"w");

	fscanf(inc,"%s %s %s %s\n",a,a,a,a);
	cout<<a;

	fprintf(out,"ply\n");
	switch(format)
	{
	case 0:fprintf(out,"format ascii 1.0\n");
		break;
	case 1:fprintf(out,"format binary_little_endian 1.0\n");
		break;
	case 3:fprintf(out,"format binary_big_endian 1.0\n");
		break;
	}
	fprintf(out,"comment made by Vishesh Gupta\n");
	fprintf(out,"comment This contains a Point Cloud\n");
	fprintf(out,"comment This is a Depth image i,j, and depth\n");
	fprintf(out,"element vertex %ld\n",640*480);
	fprintf(out,"property float x\n");
	fprintf(out,"property float y\n");
	fprintf(out,"property float z\n");
	fprintf(out,"property uchar blue\n");
	fprintf(out,"property uchar green\n");
	fprintf(out,"property uchar red\n");	
	fprintf(out,"end_header\n");
	
	double m=0.000219;
	double c=-0.00007002;
	double g=0.000052515;
	double t=0.000125;
	int r,green,b;

	//long int index=0;
	for(int y=0;y<480;y++)
		for(int x=0;x<640;x++)
		{
			fscanf(ind,"%s",raw_depth);
			Depth=atof(raw_depth);
			//Depth=Depth*10;
			//cout<<Depth;
			fscanf(inc,"%d",&r);
			fscanf(inc,"%d",&green);
			fscanf(inc,"%d",&b);
			//cout<<"\n "<<r<<green<<b;
			//if(Depth != 0)
			{
				float i=x;
				float j=y;
				float k= 8*Depth;

				TransformedVertex.x= k*(m*i/1000 + c);
				TransformedVertex.y= k*(-1*m*j/1000 + g);
				TransformedVertex.z= t*k;

				fprintf(out,"%lf %lf %lf %d %d %d\n",TransformedVertex.x,TransformedVertex.y,TransformedVertex.z,b,green,r);

			}
		}

	fclose(inc);
	fclose(ind);
	fclose(out);
}

void main()
{
	int n;
	glm::dmat4 TransformationMatix = glm::dmat4(1);
	char inputFile1[20],inputFile2[20],outputFile[20];
	cout<<"Enter the RGB file to read from\t";	cin>>inputFile1;
	cout<<"\nEnter the Depth file to read from\t";	cin>>inputFile2;
	cout<<"\nEnter the file to write into\t";	cin>>outputFile;

	cout<<"kinect 2d to 3d transformation using found matrix\n";
	
	Transform2Dto3D(inputFile1,inputFile2,outputFile,0);

	cout<<"\n\nPoint Cloud Transformation complete....\n";
	system("pause");
}
