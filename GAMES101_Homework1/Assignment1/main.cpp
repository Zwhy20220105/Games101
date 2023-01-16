#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

//get_model_matrix(float rotation_angle): 逐个元素地构建模型变换矩
//阵并返回该矩阵。在此函数中，你只需要实现三维中绕 z 轴旋转的变换矩阵，
//而不用处理平移与缩放。
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

//get_projection_matrix(float eye_fov, float aspect_ratio, float
//zNear, float zFar): 使用给定的参数逐个元素地构建透视投影矩阵并返回
//该矩阵  
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    /** arg是命令行启动时的参数，多进程！*/
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) 
        {
            filename = std::string(argv[3]);
        }
    }

    //渲染管线的功能是通过给定虚拟相机、3D场景物体以及光源等场景要素来产生或者渲染一副2D的图像。
    //图形渲染管线主要包括两个功能：
    //一,是将物体3D坐标转变为屏幕空间2D坐标
    //二,是为屏幕每个像素点进行着色。
    rst::rasterizer r(700, 700);
    Eigen::Vector3f eye_pos = {0, 0, 5};
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
   
    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        //如果是命令行的话,功能比较简单
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //设置mvp matrix
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        
        //画出图片
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        //cv::mat保存之后存下来
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imwrite(filename, image);

        return 0;
    }
    // ASCII码对应的是ESC
    while (key != 27) 
    {
        //和上面一样的基本操作,只是不保存图片
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        
        // ASCII码对应的是换行符号
        key = cv::waitKey(10);
        //统计帧率
        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
         {
            angle += 10;
        }
        else if (key == 'd')
         {
            angle -= 10;
        }
    }

    return 0;
}
