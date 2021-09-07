#include <jni.h>
#include <string>
#include <opencv2/opencv.hpp>

using namespace cv;

extern "C"
JNIEXPORT void JNICALL
Java_com_khackathon_paradise_LaneGuide_DetectionActivity_LaneDetection(JNIEnv *env, jobject thiz,
                                                                       jlong mat_addr_input,
                                                                       jlong mat_addr_result) {
    // TODO: implement LaneDetection()

    Mat &matInput = *(Mat *)mat_addr_input; // 카메라 인풋 영상
    Mat &matResult = *(Mat *)mat_addr_result; //


    UMat img_bgr, img_hsv, img_edge, img_mask;
    UMat white_mask, white_image;
    UMat yellow_mask, yellow_image;


    std::vector<Vec4i> lines;
    std::vector<std::vector<Vec4i>> separated_lines(2);
    std::vector<Point> lane(4);

    double img_size, img_center;
    double left_m, right_m;
    Point left_b, right_b;
    bool left_detect = false, right_detect = false;

    /**
     * 흰색, 노란색 영역 추출
     */
    matInput.copyTo(img_bgr);
    matInput.copyTo(matResult);

    // white range
    Scalar lower_white = Scalar(200, 200, 200);
    Scalar upper_white = Scalar(255,255,255);

    // yellow range
    Scalar lower_yellow = Scalar(10, 100, 100);
    Scalar upper_yellow = Scalar(40, 255, 255);


    cvtColor(img_bgr, img_bgr, COLOR_RGB2BGR);

    // white filter
    inRange(img_bgr, lower_white, upper_white, white_mask);
    bitwise_and(img_bgr, img_bgr, white_image, white_mask);

    cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);

    // yellow filter
    inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
    bitwise_and(img_bgr, img_bgr, yellow_image, yellow_mask);

    addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, img_edge);


    /**
     * GrayScale 변환
     */
    cvtColor(img_edge, img_edge, COLOR_RGBA2GRAY);

    /**
     * Canny Edge Detection 으로 에지 추출.
     */
    Canny(img_edge, img_edge, 50, 150);


    /**
     * region of interest 설정
     */
    int width = img_edge.cols;
    int height = img_edge.rows;

    Mat mask = Mat::zeros(height, width, CV_8UC1);

    // 관심 영역 정점 계산
    Point points[4]{
            Point(width * 0.3 / 2, height), // 아래 왼쪽
            Point(width / 2, height*0.6),
            Point(width / 2, height*0.6),
            Point(width - width * 0.3 / 2, height), // 아래 오른쪽
    };

    // 정점 내부의 색상을 그리기
    fillConvexPoly(mask, points, 4, Scalar(255, 0, 0));

    // 결과를 얻기위해 canny 이미지와 mask 를 곱한다.
    bitwise_and(img_edge, mask, img_mask);

    /**
     * hough transform
     */
    HoughLinesP(img_mask, lines, 1, CV_PI / 180, 20, 10, 20);

    matInput.copyTo(matResult);

    if (lines.size() > 0) {
        /**
         * 추출한 직선 성분으로 차선 가능성 직선을 뽑아서 좌우 각각 분리
         */
        Point p1, p2;
        std::vector<double> slopes;
        std::vector<Vec4i> final_lines, left_lines, right_lines;
        double slope_thresh = 0.3;

        // 검출된 직선들의 기울기를 계산.
        for (int i = 0; i < lines.size(); i++) {
            Vec4i line = lines[i];
            p1 = Point(line[0], line[1]);
            p2 = Point(line[2], line[3]);

            double slope;
            if (p2.x - p1.x == 0) { // 코너의 경우
                slope = 999.0;
            } else {
                slope = (p2.y - p1.y) / (double) (p2.x - p1.x);
            }

            // 기울기가 너무 수평인 선은 제외하기
            if (abs(slope) > slope_thresh) {
                slopes.push_back(slope);
                final_lines.push_back(line);
            }
        }

        // 선들을 좌우 선으로 분류
        img_center = (double) ((img_edge.cols / 2));

        for (int i = 0; i < final_lines.size(); i++) {
            p1 = Point(final_lines[i][0], final_lines[i][1]);
            p2 = Point(final_lines[i][2], final_lines[i][3]);

            if (slopes[i] > 0 && p1.x > img_center && p2.x > img_center) {
                right_detect = true;
                right_lines.push_back(final_lines[i]);
            } else if (slopes[i] < 0 && p1.x < img_center && p2.x < img_center) {
                left_detect = true;
                left_lines.push_back(final_lines[i]);
            }
        }

        separated_lines[0] = right_lines;
        separated_lines[1] = left_lines;

        /**
         * 선형 회귀로 가장 적합한 선 찾기
         */

        Point _p1, _p2, _p3, _p4;
        Vec4d left_line, right_line;
        std::vector<Point> left_points, right_points;

        if (right_detect) {
            for (auto i : separated_lines[0]) {
                _p1 = Point(i[0], i[1]);
                _p2 = Point(i[2], i[3]);

                right_points.push_back(_p1);
                right_points.push_back(_p2);
            }

            if (right_points.size() > 0) {
                //주어진 contour에 최적화된 직선 추출
                fitLine(right_points, right_line, DIST_L2, 0, 0.01, 0.1);

                right_m = right_line[1] / right_line[0];  //기울기
                right_b = Point(right_line[2], right_line[3]);
            }
        }

        if (left_detect) {
            for (auto j : separated_lines[1]) {
                _p3 = Point(j[0], j[1]);
                _p4 = Point(j[2], j[3]);

                left_points.push_back(_p3);
                left_points.push_back(_p4);
            }

            if (left_points.size() > 0) {
                //주어진 contour에 최적화된 직선 추출
                fitLine(left_points, left_line, DIST_L2, 0, 0.01, 0.01);

                left_m = left_line[1] / left_line[0];  //기울기
                left_b = Point(left_line[2], left_line[3]);
            }
        }

        //좌우 선 각각의 두 점을 계산한다.
        //y = m*x + b  --> x = (y-b) / m
        int y1 = matInput.rows;
        int y2 = matInput.rows * 0.7;

        double right_x1 = ((y1 - right_b.y) / right_m) + right_b.x;
        double right_x2 = ((y2 - right_b.y) / right_m) + right_b.x;

        double left_x1 = ((y1 - left_b.y) / left_m) + left_b.x;
        double left_x2 = ((y2 - left_b.y) / left_m) + left_b.x;

        lane[0] = Point(right_x1, y1);
        lane[1] = Point(right_x2, y2);
        lane[2] = Point(left_x1, y1);
        lane[3] = Point(left_x2, y2);

        /**
         *  영상에 최종 차선을 그린다.
         */

        std::vector<Point> poly_points;
        Mat output;
        matInput.copyTo(output);

        poly_points.push_back(lane[2]);
        poly_points.push_back(lane[0]);
        poly_points.push_back(lane[1]);
        poly_points.push_back(lane[3]);

        fillConvexPoly(output, poly_points, Scalar(0,230, 30), LINE_AA, 0);  //다각형 색 채우기
        addWeighted(output, 0.3, matInput, 0.7, 0, matInput);  //영상 합하기

        matInput.copyTo(matResult);
    }
}