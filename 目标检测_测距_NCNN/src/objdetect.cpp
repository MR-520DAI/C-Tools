#include "../include/objdetect.h"

ObjTrack::ObjTrack()
{
    ncnnanchors8_ = ncnn::Mat(6);
    ncnnanchors16_ = ncnn::Mat(6);
    ncnnanchors32_ = ncnn::Mat(6);
    ncnnanchors8_[0] = 10.f;
    ncnnanchors8_[1] = 13.f;
    ncnnanchors8_[2] = 16.f;
    ncnnanchors8_[3] = 30.f;
    ncnnanchors8_[4] = 33.f;
    ncnnanchors8_[5] = 23.f;
    ncnnanchors16_[0] = 30.f;
    ncnnanchors16_[1] = 61.f;
    ncnnanchors16_[2] = 62.f;
    ncnnanchors16_[3] = 45.f;
    ncnnanchors16_[4] = 59.f;
    ncnnanchors16_[5] = 119.f;
    ncnnanchors32_[0] = 116.f;
    ncnnanchors32_[1] = 90.f;
    ncnnanchors32_[2] = 156.f;
    ncnnanchors32_[3] = 198.f;
    ncnnanchors32_[4] = 373.f;
    ncnnanchors32_[5] = 326.f;

    ncnnyolov5_.load_param("yolov5s-kitti-sim.param");
    ncnnyolov5_.load_model("yolov5s-kitti-sim.bin");
    std::cout<<"ObjTrack Init OK!\n";
}

ObjTrack::~ObjTrack()
{
    ncnnyolov5_.clear();
    ncnnMatIN_.release();
    ncnnMatOUT_.release();
    ncnnMatINPad_.release();
    ncnnanchors8_.release();
    ncnnanchors16_.release();
    ncnnanchors32_.release();
}

int ObjTrack::detect_yolov5(const cv::Mat& cvMatImg)
{
#if _SHOW_TIME_
    struct timespec time1_all = {0, 0};
    struct timespec time2_all = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time1_all);
#endif
    /* 分辨率系数 */
    int img_w = cvMatImg.cols;
    int img_h = cvMatImg.rows;
    int w = img_w;
    int h = img_h;
    float scale = 1.f;
    if (w > h)
    {
        scale = (float)target_size_ / w;
        w = target_size_;
        h = h * scale;
    }
    else
    {
        scale = (float)target_size_ / h;
        h = target_size_;
        w = w * scale;
    }

    /* 图像预处理 */
#if _SHOW_TIME_
    struct timespec time1_img = {0, 0};
    struct timespec time2_img = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time1_img);
#endif

    ncnnMatIN_ = ncnn::Mat::from_pixels_resize(cvMatImg.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h, w, h);
    int wpad = (w + MAX_STRIDE_ - 1) / MAX_STRIDE_ * MAX_STRIDE_ - w;
    int hpad = (h + MAX_STRIDE_ - 1) / MAX_STRIDE_ * MAX_STRIDE_ - h;
    ncnn::copy_make_border(ncnnMatIN_, ncnnMatINPad_, hpad / 2, hpad - hpad / 2, wpad / 2, wpad - wpad / 2, ncnn::BORDER_CONSTANT, 114.f);
    ncnnMatINPad_.substract_mean_normalize(0, norm_vals_);

#if _SHOW_TIME_
    clock_gettime(CLOCK_REALTIME, &time2_img);
    std::cout << "img process time:" << (time2_img.tv_sec - time1_img.tv_sec)*1000 + (time2_img.tv_nsec - time1_img.tv_nsec)/1000000 << "ms\n";
#endif

    /* 推理阶段 */
    std::vector<Object> proposals;

#if _SHOW_TIME_
    clock_gettime(CLOCK_REALTIME, &time1_img);
#endif

    ncnn::Extractor ncnnex = ncnnyolov5_.create_extractor();
    
#if _SHOW_TIME_
    clock_gettime(CLOCK_REALTIME, &time2_img);
    std::cout << "create extractor time:" << (time2_img.tv_sec - time1_img.tv_sec)*1000 + (time2_img.tv_nsec - time1_img.tv_nsec)/1000000 << "ms\n";
#endif

    ncnnex.input("images", ncnnMatINPad_);

#if _SHOW_TIME_
    clock_gettime(CLOCK_REALTIME, &time1_img);
#endif
    // stride 8 解码
    {
        std::vector<Object> objects8;
        ncnnex.extract("output", ncnnMatOUT_);
        generate_proposals(ncnnanchors8_, 8, ncnnMatINPad_, ncnnMatOUT_, prob_threshold_, objects8);
        proposals.insert(proposals.end(), objects8.begin(), objects8.end());
        std::vector<Object>().swap(objects8);
        ncnnMatOUT_.release();
    }
#if _SHOW_TIME_
    clock_gettime(CLOCK_REALTIME, &time2_img);
    std::cout << "decode8 time:" << (time2_img.tv_sec - time1_img.tv_sec)*1000 + (time2_img.tv_nsec - time1_img.tv_nsec)/1000000 << "ms\n";

    clock_gettime(CLOCK_REALTIME, &time1_img);
#endif
    // stride 16 解码
    {
        std::vector<Object> objects16;
        ncnnex.extract("353", ncnnMatOUT_);
        generate_proposals(ncnnanchors16_, 16, ncnnMatINPad_, ncnnMatOUT_, prob_threshold_, objects16);
        proposals.insert(proposals.end(), objects16.begin(), objects16.end());
        std::vector<Object>().swap(objects16);
        ncnnMatOUT_.release();
    }
#if _SHOW_TIME_
    clock_gettime(CLOCK_REALTIME, &time2_img);
    std::cout << "decode16 time:" << (time2_img.tv_sec - time1_img.tv_sec)*1000 + (time2_img.tv_nsec - time1_img.tv_nsec)/1000000 << "ms\n";

    clock_gettime(CLOCK_REALTIME, &time1_img);
#endif
    // stride 32 解码
    {
        std::vector<Object> objects32;
        ncnnex.extract("367", ncnnMatOUT_);
        generate_proposals(ncnnanchors32_, 32, ncnnMatINPad_, ncnnMatOUT_, prob_threshold_, objects32);
        proposals.insert(proposals.end(), objects32.begin(), objects32.end());
        std::vector<Object>().swap(objects32);
        ncnnMatOUT_.release();
    }
#if _SHOW_TIME_
    clock_gettime(CLOCK_REALTIME, &time2_img);
    std::cout << "decode32 time:" << (time2_img.tv_sec - time1_img.tv_sec)*1000 + (time2_img.tv_nsec - time1_img.tv_nsec)/1000000 << "ms\n";
#endif

    // 按照置信度对每个目标进行排序
    qsort_descent_inplace(proposals);

    // nms筛选
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, nms_threshold_);

    int count = picked.size();
    objects_.resize(count);

    for (int i = 0; i < count; i++)
    {
        objects_[i] = proposals[picked[i]];

        // 尺度恢复
        float x0 = (objects_[i].rect.x - (wpad / 2)) / scale;
        float y0 = (objects_[i].rect.y - (hpad / 2)) / scale;
        float x1 = (objects_[i].rect.x + objects_[i].rect.width - (wpad / 2)) / scale;
        float y1 = (objects_[i].rect.y + objects_[i].rect.height - (hpad / 2)) / scale;

        // 边界处理
        x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
        y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
        x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
        y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

        objects_[i].rect.x = x0;
        objects_[i].rect.y = y0;
        objects_[i].rect.width = x1 - x0;
        objects_[i].rect.height = y1 - y0;
    }

    ncnnex.clear();
#if _SHOW_TIME_
    clock_gettime(CLOCK_REALTIME, &time2_all);
    std::cout << "all time:" << (time2_all.tv_sec - time1_all.tv_sec)*1000 + (time2_all.tv_nsec - time1_all.tv_nsec)/1000000 << "ms\n";
#endif
}

float ObjTrack::sigmoid(float x)
{
    return static_cast<float>(1.f / (1.f + exp(-x)));
}

float ObjTrack::intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

void ObjTrack::generate_proposals(const ncnn::Mat& anchors, int stride, const ncnn::Mat& in_pad, const ncnn::Mat& feat_blob, float prob_threshold, std::vector<Object>& objects)
{
    const int num_grid = feat_blob.h;

    int num_grid_x;
    int num_grid_y;
    if (in_pad.w > in_pad.h)
    {
        num_grid_x = in_pad.w / stride;
        num_grid_y = num_grid / num_grid_x;
    }
    else
    {
        num_grid_y = in_pad.h / stride;
        num_grid_x = num_grid / num_grid_y;
    }

    const int num_class = feat_blob.w - 5;

    const int num_anchors = anchors.w / 2;

    for (int q = 0; q < num_anchors; q++)
    {
        const float anchor_w = anchors[q * 2];
        const float anchor_h = anchors[q * 2 + 1];

        const ncnn::Mat feat = feat_blob.channel(q);

        for (int i = 0; i < num_grid_y; i++)
        {
            for (int j = 0; j < num_grid_x; j++)
            {
                const float* featptr = feat.row(i * num_grid_x + j);

                // find class index with max class score
                int class_index = 0;
                float class_score = -FLT_MAX;
                for (int k = 0; k < num_class; k++)
                {
                    float score = featptr[5 + k];
                    if (score > class_score)
                    {
                        class_index = k;
                        class_score = score;
                    }
                }

                float box_score = featptr[4];

                float confidence = sigmoid(box_score) * sigmoid(class_score);

                if (confidence >= prob_threshold)
                {
                    // yolov5/models/yolo.py Detect forward
                    // y = x[i].sigmoid()
                    // y[..., 0:2] = (y[..., 0:2] * 2. - 0.5 + self.grid[i].to(x[i].device)) * self.stride[i]  # xy
                    // y[..., 2:4] = (y[..., 2:4] * 2) ** 2 * self.anchor_grid[i]  # wh

                    float dx = sigmoid(featptr[0]);
                    float dy = sigmoid(featptr[1]);
                    float dw = sigmoid(featptr[2]);
                    float dh = sigmoid(featptr[3]);

                    float pb_cx = (dx * 2.f - 0.5f + j) * stride;
                    float pb_cy = (dy * 2.f - 0.5f + i) * stride;

                    float pb_w = pow(dw * 2.f, 2) * anchor_w;
                    float pb_h = pow(dh * 2.f, 2) * anchor_h;

                    float x0 = pb_cx - pb_w * 0.5f;
                    float y0 = pb_cy - pb_h * 0.5f;
                    float x1 = pb_cx + pb_w * 0.5f;
                    float y1 = pb_cy + pb_h * 0.5f;

                    Object obj;
                    obj.rect.x = x0;
                    obj.rect.y = y0;
                    obj.rect.width = x1 - x0;
                    obj.rect.height = y1 - y0;
                    obj.label = class_index;
                    obj.prob = confidence;

                    objects.push_back(obj);
                }
            }
        }
    }
}

void ObjTrack::qsort_descent_inplace(std::vector<Object>& faceobjects)
{
    if (faceobjects.empty())
        return;

    qsort_descent_inplace(faceobjects, 0, faceobjects.size() - 1);
}

void ObjTrack::qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

void ObjTrack::nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects, int id)
{
    static const char* class_names[] = {
        "car", "van", "truck", "pedestrian", "person", "cyclist", "tram", "misc", "dontcare"
    };

    cv::Mat image = bgr.clone();

    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];
        if(obj.label < 3)
        {
            // fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
            //         obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);

            cv::rectangle(image, obj.rect, cv::Scalar(0, 255, 0));
            std::cout<<"x:"<<obj.rect.x<<" y:"<<obj.rect.y<<" width:"<<obj.rect.width<<" height:"<<obj.rect.height<<std::endl;
            char text[256];
            sprintf(text, "%s %.1f", class_names[obj.label], 0.);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

            int x = obj.rect.x;
            int y = obj.rect.y - label_size.height - baseLine;
            if (y < 0)
                y = 0;
            if (x + label_size.width > image.cols)
                x = image.cols - label_size.width;

            cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                        cv::Scalar(255, 255, 255), -1);

            cv::putText(image, text, cv::Point(x, y + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
        }
        else
        {
            continue;
        }
    }

    //cv::imshow("image", image);
    //cv::waitKey(0);
    std::string savepath = "./result/" + std::to_string(id) + ".png";
    cv::imwrite(savepath.c_str(), image);
}

