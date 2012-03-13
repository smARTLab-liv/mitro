#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>


#define DEV_IN1 "/dev/video0"
#define DEV_IN2 "/dev/video1"
#define DEV_OUT "/dev/video2"

#define WIDTH_SMALL 320
#define HEIGHT_SMALL 240

#define WIDTH 640
#define HEIGHT 480

#define NUM_BUFFERS 2

typedef struct {
  void *start;
  size_t length;
} buffer;


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

  buffers = calloc(reqbuf.count, sizeof(buffer));
  assert(NULL != buffers);

  int i;
  for (i = 0; i < reqbuf.count; i++) {
    struct v4l2_buffer buf;
    memset (&buf, 0, sizeof(buf));
    buf.type = reqbuf.type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (-1 == ioctl (device, VIDIOC_QUERYBUF, &buf)) {
      perror ("VIDIOC_QUERYBUF");
      exit (EXIT_FAILURE);
    }

    // printf("buffer length: %d", buf.length);

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
  for (i = 0; i < reqbuf.count; i++) {
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

int main(int argc, char**argv)
{
  int devin1, devin2, devout;

  // input 1
  devin1 = open_device(DEV_IN1);
 
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
  devin2 = open_device(DEV_IN2);
  vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  set_resolution(devin2, WIDTH, HEIGHT, vid_format);

  buffer *buffers2;
  buffers2 = create_buffers(devin2, NUM_BUFFERS, WIDTH*HEIGHT*2);
  start_streamin(devin2);

  // output
  devout = open_device(DEV_OUT);
  vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  set_resolution(devout, WIDTH, HEIGHT, vid_format);
  

  // output buffer
  __u8*buffer; 
  size_t sizeimage = WIDTH*HEIGHT*2; //vid_format.fmt.pix.sizeimage;
  buffer=(__u8*)malloc(sizeof(__u8)*sizeimage);
  memset(buffer, 0, sizeimage);

  int counter;
  counter = 0;

  while (1) {
    counter++;
    
    // deque buffers
    struct v4l2_buffer buf1;
    buf1 = deq_buffer(devin1);

    struct v4l2_buffer buf2;
    buf2 = deq_buffer(devin2);

    memcpy(buffer, buffers1[buf1.index].start, sizeimage);
    
    int* b1;
    int* b2;
    b1 = (int*) buffers2[buf2.index].start;
    b2 = (int*) buffer;
    int x, y;
    int i0, i1, i2, i3, o0, o1, o2, o3, r;

    for (y=0; y<HEIGHT/4; y++) {
      for (x=0; x<WIDTH/8; x++) {
	
	i0 = b1[y*4*WIDTH/2 + x*4];
	i1 = b1[y*4*WIDTH/2 + x*4 + 1];
	i2 = b1[y*4*WIDTH/2 + x*4 + 2];
	i3 = b1[y*4*WIDTH/2 + x*4 + 3];

	o0 = 0; o1 = 0; o2 = 0; o3 = 0;

	int t;
	for (t=0; t<4; t++) {
	  i0 = b1[(y*4+t)*WIDTH/2 + x*4];
	  i1 = b1[(y*4+t)*WIDTH/2 + x*4 + 1];
	  i2 = b1[(y*4+t)*WIDTH/2 + x*4 + 2];
	  i3 = b1[(y*4+t)*WIDTH/2 + x*4 + 3];
	  o0 += (0x000000ff & i0) + (0x000000ff & i1) + (0x000000ff & i2) + (0x000000ff & i3); 
	  o1 += (0x000000ff & i0>>8) + (0x000000ff & i1>>8) + (0x000000ff & i2>>8) + (0x000000ff & i3>>8); 
	  o2 += (0x000000ff & i0>>16) + (0x000000ff & i1>>16) + (0x000000ff & i2>>16) + (0x000000ff & i3>>16); 
	  o3 += (0x000000ff & i0>>24) + (0x000000ff & i1>>24) + (0x000000ff & i2>>24) + (0x000000ff & i3>>24); 
	}
	r = 0x000000ff & o0 >> 4 | 
	  0x0000ff00 & (o1 >> 4) << 8 | 
	  0x00ff0000 & (o2 >> 4) << 16 | 
	  0xff000000 & (o3 >> 4) << 24;

	b2[y*WIDTH/2 + x] = r;

     }
    }
    /*
    for (y=0; y<HEIGHT/4; y++) {
      for (x=0; x<WIDTH/8; x++) {
	i0 = b1[y*WIDTH*2 + x*4];
	i1 = b1[y*WIDTH*2 + x*4 + 1];
	i2 = b1[y*WIDTH*2 + x*4 + 2];
	i3 = b1[y*WIDTH*2 + x*4 + 3];
	r = (0x000f & i0); // + 0x000f & i1 + 0x000f & i2 + 0x000f & i3) >> 2;
	r = r | (0x00f0 & i0);
	//r = r | (0x0f00 & i0);
	r = r | (0xf000 & i0);
	b2[y*WIDTH/2 + x] = r; //b1[y*WIDTH*2 + x*4] ;
      }
    }
    */
    // buffers[buf.index].start
    write(devout, buffer, sizeof(__u8) * sizeimage);
    //write(devout, buffer, sizeimage);

    // queue buffers
    q_buffer(devin1, &buf1);
    q_buffer(devin2, &buf2);


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
