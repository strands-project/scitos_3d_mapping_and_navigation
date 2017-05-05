#include "core/Camera.h"

namespace reglib
{

unsigned int camera_id_count = 0;

Camera::Camera(){
	id = camera_id_count++;

	width = 640;
	height = 480;
	fx = 535.0;
	fy = 535.0;
	cx = float(width-1)/2;
	cy = float(height-1)/2;
	idepth_scale = 0.001/5.0;

	pixel_weights = 0;
	pixel_sums = 0;
}

Camera::~Camera(){}

void Camera::save(std::string path){
	//printf("Camera::save(%s)\n",path.c_str());
	unsigned long buffersize = 7*sizeof(double);
	char* buffer = new char[buffersize];
	double * buffer_double = (double *)buffer;
	unsigned long * buffer_long = (unsigned long *)buffer;

	int counter = 0;
	buffer_long[counter++] = id;
	buffer_long[counter++] = width;
	buffer_long[counter++] = height;
	buffer_double[counter++] = fx;
	buffer_double[counter++] = fy;
	buffer_double[counter++] = cx;
	buffer_double[counter++] = cy;

	char buf [1024];
	sprintf(buf,"%s_data.txt",path.c_str());

	std::ofstream outfile (buf,std::ofstream::binary);
	outfile.write (buffer,buffersize);
	outfile.close();
	delete[] buffer;
}

void Camera::print(){
	printf("Camera fx: %5.5f fy: %5.5f cx: %5.5f cy: %5.5f\n",fx,fy,cx,cy);
}


Camera * Camera::clone(){
	Camera * cam = new Camera();
	cam->fx = fx;
	cam->fy = fy;
	cam->cx = cx;
	cam->cy = cy;
	return cam;
}

Camera * Camera::load(std::string path){
	//printf("Camera::load(%s)\n",path.c_str());
	Camera * cam = new Camera();

	std::streampos size;
	char * buffer;

	std::ifstream file (path+"_data.txt", std::ios::in | std::ios::binary | std::ios::ate);
	if (file.is_open()){
		size = file.tellg();
		buffer = new char [size];
		file.seekg (0, std::ios::beg);
		file.read (buffer, size);
		file.close();

		double *		buffer_double	= (double *)buffer;
		unsigned long * buffer_long		= (unsigned long *)buffer;

		int counter = 0;
		cam->id		= buffer_long[counter++];
		cam->width	= buffer_long[counter++];
		cam->height = buffer_long[counter++];
		cam->fx		= buffer_double[counter++];
		cam->fy		= buffer_double[counter++];
		cam->cx		= buffer_double[counter++];
		cam->cy		= buffer_double[counter++];

		camera_id_count = std::max(int(cam->id+1),int(camera_id_count));

		delete[] buffer;
	}else{std::cout << "Unable to open file" << path << std::endl;}

	return cam;
}

}
