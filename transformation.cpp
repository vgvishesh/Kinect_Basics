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

void TransformPointCloud(char *ip,char *op,glm::dmat4 matrix,int format)
{
	double *Data,position;
	long NumVertexD;
	char str[100],*substr="element",NumVertexC[15],*end_header="end_header";
	glm::dvec4 PointCloudVertex,TransformedVertex;
	FILE *in=fopen(ip,"r"),*out=fopen(op,"w");
	
	while(1)
	{
		fscanf(in,"%s\n",str);
		cout<<str<<"\n";
		if(strcmp(str,substr)==0)
		{
			fscanf(in,"%s %s\n",str,NumVertexC);
			break;
		}
	}
	NumVertexD=atof(NumVertexC);
	//cout<<NumVertexD;

	while(strcmp(str,end_header)!=0)
	{
			fscanf(in,"%s\n",str);
			 //cout<<str<<"\n";
	}

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
	fprintf(out,"element vertex %ld\n",NumVertexD);
	fprintf(out,"property float x\n");
	fprintf(out,"property float y\n");
	fprintf(out,"property float z\n");
	fprintf(out,"end_header\n");
	
	//long int index=0;

	while(!feof(in))
	{
		fscanf(in,"%lf %lf %lf\n",&PointCloudVertex.x,&PointCloudVertex.y,&PointCloudVertex.z);
		if(PointCloudVertex.z!=0)
		{
			TransformedVertex = matrix * PointCloudVertex;
			fprintf(out,"%lf %lf %lf\n",TransformedVertex.x,TransformedVertex.y,TransformedVertex.z);
		}
	}

	fclose(in);
	fclose(out);
}

void TransformPointCloudToKinectDepthImage(char *ip,char *op,int format,bool color_info)
{
	double *Data,position;
	long NumVertexD;
	char str[100],*substr="element",NumVertexC[15],*end_header="end_header";
	glm::dvec4 PointCloudVertex,TransformedVertex;
	FILE *in=fopen(ip,"r"),*out=fopen(op,"w");
	
	while(1)
	{
		fscanf(in,"%s\n",str);
		cout<<str<<"\n";
		if(strcmp(str,substr)==0)
		{
			fscanf(in,"%s %s\n",str,NumVertexC);
			break;
		}
	}
	NumVertexD=atof(NumVertexC);
	//cout<<NumVertexD;

	while(strcmp(str,end_header)!=0)
	{
			fscanf(in,"%s\n",str);
			 //cout<<str<<"\n";
	}

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
	fprintf(out,"element vertex %ld\n",NumVertexD);
	fprintf(out,"property float x\n");
	fprintf(out,"property float y\n");
	fprintf(out,"property float z\n");
	fprintf(out,"end_header\n");
	
	//long int index=0;

	// kinect transformation matrix for (i,j,k) -> (x,y,z)
	// a b c d 			i
	// e f g h 	x		j
	// 0 0 0 1			k
	//					1
	//c= -0.00007002
	//d= 0
	//a= m * k/1000
	//b= 0
	//g= 0.000052515
	//h= 0
	//e= 0
	//f= -m * k/1000
	//m= 0.000219

	if(color_info)
	{
		while(!feof(in))
		{
			fscanf(in,"%lf %lf %lf\n",&PointCloudVertex.x,&PointCloudVertex.y,&PointCloudVertex.z);
			//cout<<PointCloudVertex.x<<"\t"<<PointCloudVertex.y<<"\t"<<PointCloudVertex.z<<"\n";
			TransformedVertex.z=PointCloudVertex.z/(0.000125*8);
			float k=TransformedVertex.z * 8;
			TransformedVertex.x= (PointCloudVertex.x/k + 0.00007002 ) * (1000/0.000219);
			TransformedVertex.y= (PointCloudVertex.y/k - 0.000052515 ) * (-1000/0.000219);

			fprintf(out,"%lf %lf %lf\n",-1*TransformedVertex.x,TransformedVertex.y,TransformedVertex.z);
			//printf("%lf %lf %lf\n\n",TransformedVertex.x,TransformedVertex.y,TransformedVertex.z);
			fscanf(in,"%lf %lf %lf\n",&PointCloudVertex.x,&PointCloudVertex.y,&PointCloudVertex.z);
		}
	}
	else 
	{
		while(!feof(in))
		{
			fscanf(in,"%lf %lf %lf\n",&PointCloudVertex.x,&PointCloudVertex.y,&PointCloudVertex.z);
			//cout<<PointCloudVertex.x<<"\t"<<PointCloudVertex.y<<"\t"<<PointCloudVertex.z<<"\n";
			TransformedVertex.z=PointCloudVertex.z/(0.000125*8);
			float k=TransformedVertex.z * 8;
			TransformedVertex.x= (PointCloudVertex.x/k + 0.00007002 ) * (1000/0.000219);
			TransformedVertex.y= (PointCloudVertex.y/k - 0.000052515 ) * (-1000/0.000219);

			fprintf(out,"%lf %lf %lf\n",-1*TransformedVertex.x,TransformedVertex.y,TransformedVertex.z);
			//printf("%lf %lf %lf\n\n",TransformedVertex.x,TransformedVertex.y,TransformedVertex.z);
		}
	}

	fclose(in);
	
 	fclose(out);
}


//uses the transformation matrix to transform <i,j,k> to <x,y,>
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
void Transform2Dto3D(char *ip,char *op,int format)
{
	double *Data,position;
	long NumVertexD;
	char str[100],*substr="element",NumVertexC[15],*end_header="end_header";
	glm::dvec4 PointCloudVertex,TransformedVertex;
	FILE *in=fopen(ip,"r"),*out=fopen(op,"w");
	
	while(1)
	{
		fscanf(in,"%s\n",str);
		cout<<str<<"\n";
		if(strcmp(str,substr)==0)
		{
			fscanf(in,"%s %s\n",str,NumVertexC);
			break;
		}
	}
	NumVertexD=atof(NumVertexC);
	//cout<<NumVertexD;

	while(strcmp(str,end_header)!=0)
	{
			fscanf(in,"%s\n",str);
			 //cout<<str<<"\n";
	}

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
	fprintf(out,"element vertex %ld\n",NumVertexD);
	fprintf(out,"property float x\n");
	fprintf(out,"property float y\n");
	fprintf(out,"property float z\n");
	fprintf(out,"end_header\n");
	
	double m=0.000219;
	double c=-0.00007002;
	double g=0.000052515;
	double t=0.000125;

	//long int index=0;
	while(!feof(in))
	{
		fscanf(in,"%lf %lf %lf\n",&PointCloudVertex.x,&PointCloudVertex.y,&PointCloudVertex.z);

		if(PointCloudVertex.z != 0)
		{
			float i=-1 * PointCloudVertex.x;
			float j=PointCloudVertex.y;
			float k= PointCloudVertex.z * 8;

			TransformedVertex.x= k*(m*i/1000 + c);
			TransformedVertex.y= k*(-1*m*j/1000 + g);
			TransformedVertex.z= t*k;

			fprintf(out,"%lf %lf %lf\n",TransformedVertex.x,TransformedVertex.y,TransformedVertex.z);
		}
	}
	fclose(in);
	fclose(out);
}

void main()
{
	int n;
	glm::dmat4 TransformationMatix = glm::dmat4(1);
	char inputFile[20],outputFile[20];
	cout<<"Enter the file to read from\t";	cin>>inputFile;
	cout<<"\nEnter the file to write into\t";	cin>>outputFile;

	cout<<"\n1 -> user defined transformation matrix\n";
	cout<<"2 -> kinect 3d to 2d transformation using found matrix \n";
	cout<<"3 -> kinect 2d to 3d transformation using found matrix\n";
	cin>>n;

	if(n==1)
	{
		// user defined transformation matrix....

			/*<M0>0.999615</M0>
			<M1>0.010349</M1>
			<M2>0.025736</M2>
			<M3>-0.167867</M3>
			<M4>-0.010902</M4>
			<M5>0.999711</M5>
			<M6>0.021417</M6>
			<M7>-0.399675</M7>
			<M8>-0.025507</M8>
			<M9>-0.021689</M9>
			<M10>0.999439</M10>
			<M11>0.784497</M11>
			<M12>0.000000</M12>
			<M13>0.000000</M13>
			<M14>0.000000</M14>
			<M15>1.000000</M15>*/

		TransformationMatix[0].x=0.999615;
		TransformationMatix[0].y=-0.010902;
		TransformationMatix[0].z=-0.025507;
		TransformationMatix[0].w=0.000000;

		TransformationMatix[1].x=0.010349;
		TransformationMatix[1].y=0.999711;
		TransformationMatix[1].z=-0.021689;
		TransformationMatix[0].w=0.00000;;

		TransformationMatix[2].x=0.025736;
		TransformationMatix[2].y=0.021417;
		TransformationMatix[2].z=0.999439;
		TransformationMatix[0].w=0.00000;

		TransformationMatix[3].x=-0.167867;
		TransformationMatix[3].y=-0.399675;
		TransformationMatix[3].z=0.784497;
		TransformationMatix[0].w=1.000000;
		//.............................................
		TransformPointCloud(inputFile,outputFile,TransformationMatix,0);
	}

	else if (n==2)
	{
		bool i;
		cout<<"\nDoes this file contain color information 1: yes, 0:no\t";
		cin>>i;
		TransformPointCloudToKinectDepthImage(inputFile,outputFile,0,i);
	}

	else if(n==3)
		Transform2Dto3D(inputFile,outputFile,0);

	cout<<"\n\nPoint Cloud Transformation complete....\n";
	system("pause");
}
