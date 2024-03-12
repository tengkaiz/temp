#include <cstdio>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  //读取视频或摄像头
  VideoCapture cap(8); 

  if (!cap.isOpened()) {
    std::cout<<"无法的开摄像头"<<std::endl;
}
  cv::Mat frame;
  while (cap.read(frame)) {
      // 在这里对每一帧图像进行处理

      // imwrite("Camera.jpg", frame); 
      
     imshow("Camera", frame);  // 显示图像
      if (cv::waitKey(1) == 27) {   // 按下ESC键退出循环
          break;
      }
  }

  cap.release();
  destroyAllWindows();

  return 0;
}

