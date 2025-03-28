#include <chrono>
#include <string>

#include "visual_odometry.h"
#include "config.h"


namespace myslam {

    VO::VO(std::string &config_path): config_file_path_(config_path) {
        std::cout << "config_file_path_: " << config_file_path_ << std::endl;
    }

    bool VO::Init() {

        LOG(INFO) << "VO init";
        if(Config::SetParameterFile(config_file_path_) == false) {
            return false;
        }
        
        dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
        CHECK_EQ(dataset_->Init(), true);
        
        
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        viewer_ = Viewer::Ptr(new Viewer);
        map_ = Map::Ptr(new Map);

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCamera(dataset_->GetCamera(0), dataset_->GetCamera(1));

        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        viewer_->SetMap(map_);

        return true;
    }

    void VO::Run() {
        while(Step()) {
            LOG(INFO) << "VO is running";

        }

        // when the dataset is finished, we stop the viewer
        
        backend_->Stop();
        viewer_->Close();

        LOG(INFO) << "VO exit";
    }

    bool VO::Step() {
        
        Frame::Ptr new_frame = dataset_->NextFrame();
        if(new_frame == nullptr) return false;
        

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();

        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return success;
        // return false;
    }
}