#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "mitro_sonar/SonarScan.h"

#define WIDTH_SMALL 320
#define HEIGHT_SMALL 240

#define WIDTH 640
#define HEIGHT 480

#define NUM_BUFFERS 2

#define FIFO_FILE "/tmp/multicam-fifo" // not needed anymore
#define MAX_BUF_SIZE 255


int view = 0;
int new_view = -1;

// buffers used for V4L2
typedef struct {
  void *start;
  size_t length;
} buffer;

// used for augmentation
typedef struct {
  int x;
  int y;
} Point2D;
typedef struct {
  int Y;
  int U;
  int V;
} ColorYUV;
std::vector<Point2D> points;
std::vector<float> sonar_data(5,0);

int open_device(char* name) {
  int device;
  device = open(name, O_RDWR);
  if (device < 0) {
    printf("Failed to open video device %s.", name);
    exit(EXIT_FAILURE);
  }
  struct v4l2_capability vid_caps;
  if (-1 == ioctl(device, VIDIOC_QUERYCAP, &vid_caps)) {
    printf("Failed to get video capabilities of device %s.\n", name);
    perror("VIDIOC_QUERYCAP");
    exit(EXIT_FAILURE);
  }
  printf("Successfully opened device %s.\n", name);
  return device;
}

void set_resolution(int device, int w, int h, struct v4l2_format format) {
  format.fmt.pix.width = w;
  format.fmt.pix.height = h;
  format.fmt.pix.bytesperline = w;
  format.fmt.pix.sizeimage = w*h*2;
  if (-1 == ioctl(device, VIDIOC_S_FMT, &format)) {
    perror("VIDIOC_S_FMT");
    exit(EXIT_FAILURE);
  }
}

buffer* create_buffers(int device, unsigned int num,  size_t size) {
  buffer *buffers;
  struct v4l2_requestbuffers reqbuf;
  memset (&reqbuf, 0, sizeof (reqbuf));
  reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbuf.memory = V4L2_MEMORY_MMAP;
  reqbuf.count = num;

  if (-1 == ioctl (device, VIDIOC_REQBUFS, &reqbuf)) {
    perror("VIDIOC_REQBUFS");
    exit (EXIT_FAILURE);
  }

  if (reqbuf.count < num) {
    printf ("Could not create enough buffers (%d of %d).\n", reqbuf.count, num);
    exit (EXIT_FAILURE);
  }

  buffers = new buffer[reqbuf.count]; //calloc(reqbuf.count, sizeof(buffer));
  assert(NULL != buffers);

  for (size_t i = 0; i < reqbuf.count; i++) {
    struct v4l2_buffer buf;
    memset (&buf, 0, sizeof(buf));
    buf.type = reqbuf.type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (-1 == ioctl (device, VIDIOC_QUERYBUF, &buf)) {
      perror ("VIDIOC_QUERYBUF");
      exit (EXIT_FAILURE);
    }

    buffers[i].length = buf.length; /* remember for munmap() */

    buffers[i].start = mmap (NULL, buf.length,
			     PROT_READ | PROT_WRITE, /* recommended */
			     MAP_SHARED,             /* recommended */
			     device, buf.m.offset);

    if (MAP_FAILED == buffers[i].start) {
      /* If you do not exit here you should unmap() and free()
	 the buffers mapped so far. */
      perror ("mmap");
      exit (EXIT_FAILURE);
    }
  }
  // queue all buffers
  for (size_t i = 0; i < reqbuf.count; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (-1 == ioctl(device, VIDIOC_QBUF, &buf)) {
      perror("VIDIOC_QBUF");
    } else {
      // printf("Q buffer #%d.\n", buf.index);
    }
  }
  return buffers;
}

void kill_buffers(buffer *buffers, int nbuffers){
  int i;
  for (i = 0; i < nbuffers; i++)
     munmap(buffers[i].start, buffers[i].length);
}

void start_streamin(int device) {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl (device, VIDIOC_STREAMON, &type)) {
    perror ("VIDIOC_STREAMON");
    exit(EXIT_FAILURE);
  }
}

void stop_streamin(int device) {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl (device, VIDIOC_STREAMOFF, &type)) {
    perror ("VIDIOC_STREAMOFF");
    exit(EXIT_FAILURE);
  }
}

struct v4l2_buffer deq_buffer(int device) {
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
    
  if (-1 == ioctl(device, VIDIOC_DQBUF, &buf)) {
    perror("VIDIOC_DQBUF");
  } else {
    //    printf("DEQ buffer #%d.\n", buf.index);
  }
  return buf;
}


void q_buffer(int device, struct v4l2_buffer *buf) {
  if (-1 == ioctl(device, VIDIOC_QBUF, buf)) {
    perror("VIDIOC_QBUF");
  } else {
    //    printf("Q buffer #%d.\n", buf->index);
  }
}



Point2D proj(float x, float y, float z) {
  //""" 3D to 2D projection """
  // ROS to OpenCV
  float temp = x;
  x = -(y - 0.051);
  y = -(z - 1.290);
  z = temp;
  
  float th = 63.9 / 180.0 * M_PI;
  float t_y = y;
  float t_z = z;
  // rotation
  y = cos(th) * t_y - sin(th) * t_z;
  z = sin(th) * t_y + cos(th) * t_z;
  
  float k1 = -0.286767;
  float k2 = 0.059882;
  float k3 = 0;
  float p1 = 0.000601;
  float p2 = -0.001703;

  float fx = 355.027391;
  float fy = 353.784702;
  float cx = 306.290598;
  float cy = 260.086688;

  float x_p, y_p, x_pp, y_pp;
  x_p = x / z;
  y_p = y / z;
  float r2 = x_p * x_p + y_p * y_p;

  x_pp = x_p * (1 + k1 * r2 + k2 * (r2 * r2) + k3 * (r2 * r2 * r2)) + 2 * p1 * x_p * y_p + p2 * (r2 + 2 * x_p * x_p);
  y_pp = y_p * (1 + k1 * r2 + k2 * (r2 * r2) + k3 * (r2 * r2 * r2)) + p1 * (r2 + 2 * y_p * y_p) + 2 * p2 * x_p * y_p;
  
  Point2D p;
  p.x = fx * x_pp + cx;
  p.y = fy * y_pp + cy;
  return p;
}

void line(int* buffer, int x1, int x2, int y, ColorYUV color) {
  //""" files horizontal line in a v4l2 buffer """
  if (x2 < x1) {
    int t = x1;
    x1 = x2;
    x2 = 1;
  }
  int x;
  for (x = x1/2; x <= x2/2; x++) {
    int Y0 = buffer[y*WIDTH/2 + x] & 0x000000ff;
    int U0 = buffer[y*WIDTH/2 + x]>>8 & 0x000000ff;
    int Y1 = buffer[y*WIDTH/2 + x]>>16 & 0x000000ff;
    int V0 = buffer[y*WIDTH/2 + x]>>24 & 0x000000ff;
    U0 = (color.U + U0) / 2;
    V0 = (color.V + V0) / 2;
    if (x*2 < x1)
       Y1 = (color.Y + Y0) / 2;
    else if (x*2 >= x2)
       Y0 = (color.Y + Y1) / 2;
    else {
      Y0 = (color.Y + Y0) / 2;
      Y1 = (color.Y + Y1) / 2;
    }
    buffer[y*WIDTH/2 + x] = V0 << 24 | Y1 << 16 | U0 << 8 | Y0; 
  }
}

void fill_rect(int* buffer, int x, int y, int width, int height, ColorYUV color) {
  for (int i = y; i <= y + height; i++) {
    line(buffer, x, x + width, i, color);
  }
}

void fill_poly(int* buffer, std::vector<Point2D> points, ColorYUV color) {
  //""" fills a polygon by calling the horizontal line fill method """
  int n = points.size();
  if (n < 2) 
    return;
  points.push_back(points[0]);
  int dx, dy;
  float slope[n];
  int xi[n];
  for (int i = 0; i < n; i ++) {
    dy = points[i+1].y - points[i].y;
    dx = points[i+1].x - points[i].x;
    if (dy == 0)
      slope[i] = 1.0;
    if (dx == 0)
      slope[i] = 0.0;
    if((dy!=0)&&(dx!=0)) {
      slope[i]=(float) dx/dy;
    }
  }
  for(int y=0;y<HEIGHT;y++) {
    int k=0;
    for(int i=0;i<n;i++) {
      if( ((points[i].y<=y)&&(points[i+1].y>y))||
	  ((points[i].y>y)&&(points[i+1].y<=y)) )
	{
	  xi[k]=(int)(points[i].x+slope[i]*(y-points[i].y));
	  k++;
	}
    }
    
    
    for(int j=0;j<k-1;j++) /*- Arrange x-intersections in order -*/
      for(int i=0;i<k-1;i++)
	{
	  if(xi[i]>xi[i+1])
	    {
	      int temp=xi[i];
	      xi[i]=xi[i+1];
	      xi[i+1]=temp;
	    }
	}
    
    for(int i=0;i<k;i+=2)
      line(buffer, xi[i],xi[i+1]+1,y, color);
  }      

}

std::vector<Point2D> twist2poly(float lin, float ang) {
  // computing the polygon for a twist
  std::vector<Point2D> points;
  std::vector<Point2D> points_temp;
  Point2D p;

  int steps = 25;
  float t_end = 3.0;

  float th = 0;
  float x = 0;
  float y = 0;
  float r = 0.21;
  points.push_back(proj(x + r * sin(-(M_PI-th)), y - r * cos(-(M_PI-th)), 0));
  points_temp.insert(points_temp.begin(), proj(x - r * sin(-(M_PI-th)), y + r * cos(-(M_PI-th)), 0));
  for (float t=0.0; t <= t_end; t+=t_end/steps) {
    x += lin * cos(th) * t_end/steps;
    y += lin * sin(th) * t_end/steps;
    th += ang * t_end/steps;
    points.push_back(proj(x + r * sin(-(M_PI-th)), y - r * cos(-(M_PI-th)), 0));
    points_temp.insert(points_temp.begin(), proj(x - r * sin(-(M_PI-th)), y + r * cos(-(M_PI-th)), 0));
  }
  points.insert(points.end(), points_temp.begin(), points_temp.end());
  return points;
}


void view_callback(const std_msgs::Int16::ConstPtr& msg)
{
  new_view = msg->data;
}

void twist_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  points = twist2poly(msg->linear.x, msg->angular.z);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  points = twist2poly(msg->twist.twist.linear.x, msg->twist.twist.angular.z);
}

void sonar_callback(const mitro_sonar::SonarScan::ConstPtr& msg) {
  sonar_data.clear();
  sonar_data.reserve(msg->ranges.size());
  std::copy(msg->ranges.begin(), msg->ranges.end(), sonar_data.begin());
}

ColorYUV rgb_to_yuv(int R, int G, int B) {
  // converting from RGB to YUV
  int Y = ((66 * R + 129 * G + 25 * B + 128) >> 8) + 16;
  int U = ((-38 * R + -74 * G + 112 * B + 128) >> 8) + 128;
  int V = ((112 * R + -94 * G + -18 * B + 128) >> 8) + 128;

  ColorYUV color;
  color.Y = Y;
  color.U = U;
  color.V = V;
  
  return color;
}


int main(int argc, char**argv)
{
  // overlay
  int R = 200; //122;
  int G = 255; //189;
  int B = 200; //255;

  ColorYUV path_color = rgb_to_yuv(R, G, B);
  
  R = 255;
  G = 50;
  B = 0;
  
  ColorYUV sonar_color = rgb_to_yuv(R, G, B); 
  
  

  if ( argc < 4 ) {
    printf("usage: %s video_in_1 video_in_2 video_out\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  char* devin1_name = argv[1];
  char* devin2_name = argv[2];
  char* devout_name = argv[3];

  ros::init(argc, argv, "multicam");
  ros::NodeHandle private_n("~");

  ros::Subscriber sub_view = private_n.subscribe("view", 1, view_callback);
  // ros::NodeHandle n;
  // ros::Subscriber sub_twist = n.subscribe("cmd_twist_mixed", 1, twist_callback);

  ros::NodeHandle n;
  ros::Subscriber sub_odom = n.subscribe("odom", 1, odom_callback);
  ros::Subscriber sub_sonar = n.subscribe("sonar_scan", 10, sonar_callback);

  // device handles
  int devin1, devin2, devout;

  // input 1
  devin1 = open_device(devin1_name);
 
  // TODO this is a hack to get the standart values in the format
  struct v4l2_format vid_format;
  memset(&vid_format, 0, sizeof(vid_format));
  vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(devin1, VIDIOC_G_FMT, &vid_format)) {
    perror("VIDIOC_G_FMT");
    exit(EXIT_FAILURE);
  }
  set_resolution(devin1, WIDTH, HEIGHT, vid_format);
  buffer *buffers1;
  buffers1 = create_buffers(devin1, NUM_BUFFERS, WIDTH*HEIGHT*2);
  start_streamin(devin1);

  // input 2
  devin2 = open_device(devin2_name);
  vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  set_resolution(devin2, WIDTH_SMALL, HEIGHT_SMALL, vid_format);
  //set_resolution(devin2, WIDTH, HEIGHT, vid_format);
  
  buffer *buffers2;
  buffers2 = create_buffers(devin2, NUM_BUFFERS, WIDTH_SMALL*HEIGHT_SMALL*2);
  start_streamin(devin2);

  // output
  devout = open_device(devout_name);
  vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  set_resolution(devout, WIDTH, HEIGHT, vid_format);

  // output buffer
  void* buffer; 
  size_t sizeimage = WIDTH*HEIGHT*2; //vid_format.fmt.pix.sizeimage;
  buffer = malloc(sizeimage);
  memset(buffer, 0, sizeimage);




  while (ros::ok()) {
    int t = new_view;
    if (t > -1 && t < 4 && t != view) { // check if view change triggers resolution change (0 to 1, and 2 to 3 are save)
      if ( t < 2 && view >= 2 ) {
	stop_streamin(devin1);
	stop_streamin(devin2);

	kill_buffers(buffers1, NUM_BUFFERS);
	kill_buffers(buffers2, NUM_BUFFERS);

	close(devin1);
	close(devin2);

	usleep(1000); // this is a heuristic, can't immediately open device again

	devin1 = open_device(devin1_name);
	devin2 = open_device(devin2_name);


	memset(&vid_format, 0, sizeof(vid_format));
	vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == ioctl(devin1, VIDIOC_G_FMT, &vid_format)) {
	  perror("VIDIOC_G_FMT");
	  exit(EXIT_FAILURE);
	}

	set_resolution(devin1, WIDTH, HEIGHT, vid_format);
	buffers1 = create_buffers(devin1, NUM_BUFFERS, WIDTH*HEIGHT*2);
 
	set_resolution(devin2, WIDTH_SMALL, HEIGHT_SMALL, vid_format);
	buffers2 = create_buffers(devin2, NUM_BUFFERS, WIDTH_SMALL*HEIGHT_SMALL*2);

	usleep(1000);
	  
	start_streamin(devin1);
	start_streamin(devin2);
      } else if ( t >= 2 && view < 2) {
	stop_streamin(devin1);
	stop_streamin(devin2);
	kill_buffers(buffers1, NUM_BUFFERS);
	kill_buffers(buffers2, NUM_BUFFERS);

	close(devin1);
	close(devin2);

	usleep(1000);

	devin1 = open_device(devin1_name);
	devin2 = open_device(devin2_name);

	memset(&vid_format, 0, sizeof(vid_format));
	vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == ioctl(devin1, VIDIOC_G_FMT, &vid_format)) {
	  perror("VIDIOC_G_FMT");
	  exit(EXIT_FAILURE);
	}

	set_resolution(devin1, WIDTH_SMALL, HEIGHT_SMALL, vid_format);
	buffers1 = create_buffers(devin1, NUM_BUFFERS, WIDTH_SMALL*HEIGHT_SMALL*2);

	set_resolution(devin2, WIDTH, HEIGHT, vid_format);
	buffers2 = create_buffers(devin2, NUM_BUFFERS, WIDTH*HEIGHT*2);

	usleep(1000);
	  
	start_streamin(devin1);
	start_streamin(devin2);
      }
      view = t;
      printf("Switched to view: %d.\n", view);
      new_view = -1;
    }

    // deque buffers
    struct v4l2_buffer buf1;
    struct v4l2_buffer buf2;
    buf1 = deq_buffer(devin1);
    buf2 = deq_buffer(devin2);

    void *largeframe;
    void *smallframe;

    if ( 0 == view ) {
      largeframe = buffers1[buf1.index].start;
      smallframe = buffers2[buf2.index].start;
    } else if ( 1 == view ) {
      largeframe = buffers1[buf1.index].start;
      smallframe = NULL;
    } else if ( 2 == view ) {
      largeframe = buffers2[buf2.index].start;
      smallframe = buffers1[buf1.index].start;
    } else if ( 3 == view ) {
      largeframe = buffers2[buf2.index].start;
      smallframe = NULL;
    }

    if ( NULL != largeframe )
      memcpy(buffer, largeframe, WIDTH*HEIGHT*2);


    if (view > 1) {
      fill_poly((int*) buffer, points, path_color);
      
      int sonar_width = 20;
      double sonar_dist_offset = 0.225;
      double sonar_dist_max = 1.0;
      double sonar_scale = 100.0;
      int sonar_pos_x[5] = {250, 280, 310, 340, 370};
      int sonar_pos_y = 470;
      
      for (int i = 0; i < 5; i++) {
        int height = std::floor(sonar_scale * (sonar_dist_max - std::min(sonar_dist_max, std::max(0.01, sonar_data[i] - sonar_dist_offset))));
        //std::cout << i << ", " << sonar_data[i] << ", " << height << std::endl; 
	fill_rect((int*) buffer, sonar_pos_x[i], sonar_pos_y - height, sonar_width, height, sonar_color);
      }
    }
  
    if ( NULL != smallframe ) { // if we have a smallframe, downsampling buffer by factor 2 
      int* b1;
      int* b2;
      b1 = (int*) smallframe;
      b2 = (int*) buffer;
      int x, y;
      int i0, i1, i2, i3, o0, o1, o2, o3, o4,  r;

      for (y=0; y<HEIGHT_SMALL; y+=2) {
	for (x=0; x<WIDTH_SMALL/2; x+=2) {
	
	  i0 = b1[y*WIDTH_SMALL/2 + x];
	  i1 = b1[y*WIDTH_SMALL/2 + x + 1];
	  i2 = b1[(y+1)*WIDTH_SMALL/2 + x];
	  i3 = b1[(y+1)*WIDTH_SMALL/2 + x + 1];

	  o0 = (0x000000ff & i0) + (0x000000ff & i1) + (0x000000ff & i2) + (0x000000ff & i3); 
	  o1 = (0x000000ff & i0>>8) + (0x000000ff & i1>>8) + (0x000000ff & i2>>8) + (0x000000ff & i3>>8); 
	  o2 = (0x000000ff & i0>>16) + (0x000000ff & i1>>16) + (0x000000ff & i2>>16) + (0x000000ff & i3>>16); 
	  o3 = (0x000000ff & i0>>24) + (0x000000ff & i1>>24) + (0x000000ff & i2>>24) + (0x000000ff & i3>>24); 

	  r = 0x000000ff & o0 >> 2 | 
	    0x0000ff00 & (o1 >> 2) << 8 | 
	    0x00ff0000 & (o2 >> 2) << 16 | 
	    0xff000000 & (o3 >> 2) << 24;

	  b2[y/2*WIDTH/2 + x/2] = r;
	}
      }
    }

    write(devout, buffer, sizeimage); // write the ouput buffer to device

    q_buffer(devin1, &buf1); // requeuing buffers
    q_buffer(devin2, &buf2);

    ros::spinOnce();
  }

  /* Cleanup. */

  stop_streamin(devin1);
  kill_buffers(buffers1, NUM_BUFFERS);
  close(devin1);

  stop_streamin(devin2);
  kill_buffers(buffers2, NUM_BUFFERS);
  close(devin2);  

  close(devout);

  return 0;
}
